#!/usr/bin/python
import rospkg
import smach
import smach_ros
import tf
import yaml
from pyassimp import pyassimp
from copy import copy
from re import findall

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.msg import RobotTrajectory
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import InteractiveMarkerControl, Marker
from dynamic_reconfigure.server import Server
from cob_grasping.cfg import parameterConfig

from simple_script_server import *

sss = simple_script_server()
mgc_left = MoveGroupCommander("arm_left")
mgc_right = MoveGroupCommander("arm_right")

planning_scene = PlanningScene()
planning_scene.is_diff = True

planning_scene_interface = PlanningSceneInterface()
pub_planning_scene = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)

abort_execution = False


def smooth_cartesian_path(traj):

    time_offset = 200000000  # 0.2s

    for i in xrange(len(traj.joint_trajectory.points)):
        traj.joint_trajectory.points[i].time_from_start += rospy.Duration(0, time_offset)

    traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(0, time_offset)

    return traj


def fix_velocities(traj):
    # fix trajectories to stop at the end
    traj.joint_trajectory.points[-1].velocities = [0]*7

    # fix trajectories to be slower
    speed_factor = 1.0
    for i in xrange(len(traj.joint_trajectory.points)):
        traj.joint_trajectory.points[i].time_from_start *= speed_factor

    return traj


def scale_joint_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = RobotTrajectory()

    # Initialize the new trajectory to be the same as the planned trajectory
    new_traj.joint_trajectory = traj.joint_trajectory

    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)

    # Store the trajectory points
    points = list(traj.joint_trajectory.points)

    # Cycle through all points and scale the time from start, speed and acceleration
    for i in xrange(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in xrange(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj


def plan_movement(planer, arm, pose, speed):
        (config, error_code) = sss.compose_trajectory("arm_" + arm, pose)
        if error_code != 0:
            rospy.logerr("Unable to parse " + pose + " configuration")

        start_state = RobotState()
        start_state.joint_state.name = config.joint_names

        start_state.joint_state.position = planer.get_current_joint_values()
        planer.set_start_state(start_state)

        planer.clear_pose_targets()
        planer.set_joint_value_target(config.points[0].positions)

        plan = planer.plan()

        plan = smooth_cartesian_path(plan)
        plan = scale_joint_trajectory_speed(plan, speed)
        return plan


def add_remove_object(co_operation, co_object, co_position, co_type):
        if co_operation == "add":
            co_object.operation = CollisionObject.ADD
            pose = Pose()
            pose.position.x = co_position[0]
            pose.position.y = co_position[1]
            pose.position.z = co_position[2]
            pose.orientation.x = co_position[3]
            pose.orientation.y = co_position[4]
            pose.orientation.z = co_position[5]
            pose.orientation.w = co_position[6]

            if co_type == "mesh":
                co_object.mesh_poses.append(pose)
            elif co_type == "primitive":
                co_object.primitive_poses.append(pose)
        elif co_operation == "remove":
            co_object.operation = CollisionObject.REMOVE
            planning_scene.world.collision_objects[:] = []
        else:
            rospy.logerr("Invalid command")
            return

        planning_scene.world.collision_objects.append(co_object)
        pub_planning_scene.publish(planning_scene)
        rospy.sleep(0.1)


class SceneManager(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'exit'],
                             input_keys=['active_arm', 'arm_positions'],
                             output_keys=['arm_positions', 'switch_arm', 'object'])

        # ---- GET PARAMETER FROM SERVER ----
        self.scenario = rospy.get_param(str(rospy.get_name()) + "/scene")
        self.switch_arm = rospy.get_param(str(rospy.get_name()) + "/switch_arm")
        self.wait_for_user = rospy.get_param(str(rospy.get_name()) + "/wait_for_user")
        self.object_dimensions = rospy.get_param(str(rospy.get_name()) + "/object_dimensions")
        self.spawn_obstacles = rospy.get_param(str(rospy.get_name()) + "/load_obstacles")
        self.planning_method = rospy.get_param(str(rospy.get_name()) + "/planning_method")

        self.path = rospkg.RosPack().get_path("cob_grasping") + "/config/scene_config.yaml"

        if rospy.get_param(str(rospy.get_name()) + "/planning_method") == "cartesian_linear":
            self.use_waypoints = True
        else:
            self.use_waypoints = False

        self.server = InteractiveMarkerServer("grasping_targets")
        self.menu_handler = MenuHandler()

        # ---- LOAD SCENE DATA ----
        self.data = self.load_data(self.path)
        self.positions_changed = False
        self.environment_changed = False

        # ---- BUILD MENU ----
        self.menu_handler.insert("Start execution", callback=self.start_planning)
        self.menu_handler.insert("Stopp execution", callback=self.stop_planning)

        # --- ENVIRONMENT MENU ---
        env_entry = self.menu_handler.insert("Environment")

        env_menu_id = 4
        for item in sorted(self.data):
            if item == self.scenario:
                self.menu_handler.setCheckState(self.menu_handler.insert(item, callback=self.change_environment,
                                                                         parent=env_entry), MenuHandler.CHECKED)
                self.last_env = env_menu_id
            else:
                self.menu_handler.setCheckState(self.menu_handler.insert(item, callback=self.change_environment,
                                                                         parent=env_entry), MenuHandler.UNCHECKED)
            env_menu_id += 1

        # --- WAYPOINT MENU ---
        if self.use_waypoints:
            wp_entry = self.menu_handler.insert("Waypoints")
            self.menu_handler.insert("Delete waypoint", parent=wp_entry, callback=self.delete_waypoint)
            self.wp_arm_right = self.menu_handler.insert("Arm right", parent=wp_entry)
            self.menu_handler.insert("Add waypoint", parent=self.wp_arm_right, callback=self.add_waypoint_right)

            self.wp_arm_left = self.menu_handler.insert("Arm left", parent=wp_entry)
            self.menu_handler.insert("Add waypoint", parent=self.wp_arm_left, callback=self.add_waypoint_left)

        # --- SWITCH ARM ---
        if self.switch_arm:
            self.menu_handler.setCheckState(self.menu_handler.insert("Switch arm", callback=self.switch_arm_callback),
                                            MenuHandler.CHECKED)
        elif not self.switch_arm:
            self.menu_handler.setCheckState(self.menu_handler.insert("Switch arm", callback=self.switch_arm_callback),
                                            MenuHandler.UNCHECKED)

        # --- EXIT ---
        self.menu_handler.insert("Exit", callback=self.exit_program)
        self.exit = False

        self.start_manipulation = threading.Event()

    def execute(self, userdata):
        self.spawn_environment()
        rospy.loginfo("Click start to continue...")
        if self.wait_for_user:
            self.start_manipulation.wait()
            self.start_manipulation.clear()

        if self.exit:
            return "exit"

        if self.planning_method != "joint":
            # ---- POSITIONS FOR RIGHT ARM ----
            waypoints_r = []
            if len(self.data[self.scenario]["positions"]["waypoints_r"]) != 0:
                for item in self.data[self.scenario]["positions"]["waypoints_r"]:
                    waypoints_r.append(Point(item[0], item[1], item[2]))
            userdata.arm_positions["right"] = {"start": Point(self.data[self.scenario]["positions"]["start_r"][0],
                                                              self.data[self.scenario]["positions"]["start_r"][1],
                                                              self.data[self.scenario]["positions"]["start_r"][2]),
                                               "waypoints": waypoints_r,
                                               "goal": Point(self.data[self.scenario]["positions"]["goal_r"][0],
                                                             self.data[self.scenario]["positions"]["goal_r"][1],
                                                             self.data[self.scenario]["positions"]["goal_r"][2])}

            # ---- POSITIONS FOR LEFT ARM ----
            waypoints_l = []
            if len(self.data[self.scenario]["positions"]["waypoints_l"]) != 0:
                for item in self.data[self.scenario]["positions"]["waypoints_l"]:
                    waypoints_l.append(Point(item[0], item[1], item[2]))
            userdata.arm_positions["left"] = {"start": Point(self.data[self.scenario]["positions"]["goal_r"][0],
                                                             self.data[self.scenario]["positions"]["goal_r"][1],
                                                             self.data[self.scenario]["positions"]["goal_r"][2]),
                                              "waypoints": waypoints_l,
                                              "goal": Point(self.data[self.scenario]["positions"]["goal_l"][0],
                                                            self.data[self.scenario]["positions"]["goal_l"][1],
                                                            self.data[self.scenario]["positions"]["goal_l"][2])}
        else:
            # ---- POSITIONS FOR RIGHT ARM ----
            joints_r = []
            if len(self.data[self.scenario]["joints"]["right"]) != 0:
                for item in self.data[self.scenario]["joints"]["right"]:
                    joints_r.append(item)
            userdata.arm_positions["right"] = {"joints": joints_r}

            # ---- POSITIONS FOR LEFT ARM ----
            joints_l = []
            if len(self.data[self.scenario]["joints"]["left"]) != 0:
                for item in self.data[self.scenario]["joints"]["left"]:
                    joints_l.append(item)
            userdata.arm_positions["left"] = {"joints": joints_l}

        # ---- SWITCH ARM ----
        userdata.switch_arm = self.switch_arm

        # ---- SAVE POSITIONS ----
        if self.positions_changed or self.environment_changed:
            self.save_data(self.path)
            self.positions_changed = False
            self.environment_changed = False

        # ---- OBJECT DIMENSIONS ----
        userdata.object = self.object_dimensions

        return "succeeded"

    @staticmethod
    def load_data(filename):
        rospy.loginfo("Reading data from yaml file...")

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)

        return doc

    def save_data(self, filename):
        rospy.loginfo("Writing data to yaml file...")
        stream = file(filename, 'w')
        yaml.dump(self.data, stream)

    @staticmethod
    def load_mesh(filename, scale):

        scene = pyassimp.load(filename)
        if not scene.meshes:
            rospy.logerr('Unable to load mesh')
            return

        mesh = Mesh()
        for face in scene.meshes[0].faces:
            triangle = MeshTriangle()
            if len(face.indices) == 3:
                triangle.vertex_indices = [face.indices[0], face.indices[1], face.indices[2]]
            mesh.triangles.append(triangle)
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale
            point.y = vertex[1] * scale
            point.z = vertex[2] * scale
            mesh.vertices.append(point)
        pyassimp.release(scene)

        return mesh

    def make_box(self, color):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale.x = self.object_dimensions[0]  # Diameter in x
        marker.scale.y = self.object_dimensions[1]  # Diameter in y
        marker.scale.z = self.object_dimensions[2]  # Height
        marker.color = color

        return marker

    def make_boxcontrol(self, msg, color):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_box(color))
        msg.controls.append(control)
        return control

    def make_marker(self, name, color, interaction_mode, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position

        int_marker.name = name
        int_marker.description = name

        # Insert a box
        self.make_boxcontrol(int_marker, color)
        int_marker.controls[0].interaction_mode = interaction_mode

        self.server.insert(int_marker, self.process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)

    def process_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if feedback.marker_name == "right_arm_start":
                self.data[self.scenario]["positions"]["start_r"] = [feedback.pose.position.x,
                                                                    feedback.pose.position.y,
                                                                    feedback.pose.position.z]
            elif feedback.marker_name == "right_arm_goal":
                self.data[self.scenario]["positions"]["goal_r"] = [feedback.pose.position.x,
                                                                   feedback.pose.position.y,
                                                                   feedback.pose.position.z]
            elif feedback.marker_name == "left_arm_goal":
                self.data[self.scenario]["positions"]["goal_l"] = [feedback.pose.position.x,
                                                                   feedback.pose.position.y,
                                                                   feedback.pose.position.z]
            elif "waypoint_r" in feedback.marker_name:
                numbers = []
                for s in feedback.marker_name:
                    numbers = findall("[-+]?\d+[\.]?\d*", s)
                number = int(numbers[0]) - 1
                self.data[self.scenario]["positions"]["waypoints_r"][number] = [feedback.pose.position.x,
                                                                                feedback.pose.position.y,
                                                                                feedback.pose.position.z]
            elif "waypoint_l" in feedback.marker_name:
                numbers = []
                for s in feedback.marker_name:
                    numbers = findall("[-+]?\d+[\.]?\d*", s)
                number = int(numbers[0]) - 1
                self.data[self.scenario]["positions"]["waypoints_l"][number] = [feedback.pose.position.x,
                                                                                feedback.pose.position.y,
                                                                                feedback.pose.position.z]

            rospy.loginfo("Position " + feedback.marker_name + ": x = " + str(feedback.pose.position.x)
                          + " | y = " + str(feedback.pose.position.y) + " | z = " + str(feedback.pose.position.z))
            self.server.applyChanges()

            self.positions_changed = True

    def add_waypoint_right(self, feedback):
        name = "waypoint_r" + str(len(self.data[self.scenario]["positions"]["waypoints_r"]) + 1)
        position = Point(1.0, 0.0, 1.0)

        color = ColorRGBA(0.0, 0.0, 1.0, 1.0)

        self.data[self.scenario]["positions"]["waypoints_r"].append(position)

        # Add marker to scene
        self.make_marker(name, color, InteractiveMarkerControl.MOVE_3D, position)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        rospy.loginfo("Added waypoint '" + str(name) + "' at position: x: " + str(position.x) + " | y: "
                      + str(position.y) + " | z: " + str(position.z))

    def add_waypoint_left(self, feedback):
        name = "waypoint_l" + str(len(self.data[self.scenario]["positions"]["waypoints_l"]) + 1)
        position = Point(1.0, 0.0, 1.0)

        color = ColorRGBA(0.0, 0.0, 1.0, 1.0)

        self.data[self.scenario]["positions"]["waypoints_l"].append(position)

        # Add marker to scene
        self.make_marker(name, color, InteractiveMarkerControl.MOVE_3D, position)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        rospy.loginfo("Added waypoint '" + str(name) + "' at position: x: " + str(position.x) + " | y: "
                      + str(position.y) + " | z: " + str(position.z))

    def delete_waypoint(self, feedback):
        name = feedback.marker_name
        numbers = []

        if "waypoint_r" in name:

            # Delete all waypoints
            for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_r"])):
                self.server.erase("waypoint_r" + str(i + 1))
            self.server.applyChanges()

            # Delete selected waypoint from list
            for s in name:
                numbers = findall("[-+]?\d+[\.]?\d*", s)
            number = int(numbers[0]) - 1
            del self.data[self.scenario]["positions"]["waypoints_r"][number]

            # Build remaining waypoints
            color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
            if len(self.data[self.scenario]["positions"]["waypoints_r"]) != 0:
                for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_r"])):
                    self.make_marker("waypoint_r" + str(i + 1), color, InteractiveMarkerControl.MOVE_3D,
                                     Point(self.data[self.scenario]["positions"]["waypoints_r"][i][0],
                                           self.data[self.scenario]["positions"]["waypoints_r"][i][1],
                                           self.data[self.scenario]["positions"]["waypoints_r"][i][2]))

            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            rospy.loginfo("Deleted waypoint '" + str(name) + "'")
        elif "waypoint_l" in name:

            # Delete all waypoints
            for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_l"])):
                self.server.erase("waypoint_l" + str(i + 1))
            self.server.applyChanges()

            # Delete selected waypoint from list
            for s in name:
                numbers = findall("[-+]?\d+[\.]?\d*", s)
            number = int(numbers[0]) - 1
            del self.data[self.scenario]["positions"]["waypoints_l"][number]

            # Build remaining waypoints
            color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
            if len(self.data[self.scenario]["positions"]["waypoints_l"]) != 0:
                for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_l"])):
                    self.make_marker("waypoint_l" + str(i + 1), color, InteractiveMarkerControl.MOVE_3D,
                                     Point(self.data[self.scenario]["positions"]["waypoints_l"][i][0],
                                           self.data[self.scenario]["positions"]["waypoints_l"][i][1],
                                           self.data[self.scenario]["positions"]["waypoints_l"][i][2]))

            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            rospy.loginfo("Deleted waypoint '" + str(name) + "'")
        else:
            rospy.logerr("Only waypoints can be deleted!")

    def start_planning(self, feedback):
        self.start_manipulation.set()

    def stop_planning(self, feedback):
        self.wait_for_user = True
        global abort_execution
        abort_execution = True

    def exit_program(self, feedback):
        self.exit = True
        self.start_manipulation.set()

    def switch_arm_callback(self, feedback):
        if self.menu_handler.getCheckState(feedback.menu_entry_id) == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(feedback.menu_entry_id, MenuHandler.CHECKED)
            self.switch_arm = True
        else:
            self.menu_handler.setCheckState(feedback.menu_entry_id, MenuHandler.UNCHECKED)
            self.switch_arm = False

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def change_environment(self, feedback):
        if self.menu_handler.getCheckState(feedback.menu_entry_id) == MenuHandler.UNCHECKED:
            self.menu_handler.setCheckState(self.last_env, MenuHandler.UNCHECKED)
            self.menu_handler.setCheckState(feedback.menu_entry_id, MenuHandler.CHECKED)
            self.last_env = feedback.menu_entry_id
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            # Set new scenario
            self.scenario = self.menu_handler.getTitle(feedback.menu_entry_id)

            # Spawn new environment
            self.spawn_environment()

    def spawn_environment(self):
        # Clear environment
        self.clear_environment()

        # Spawn environment
        rospy.loginfo("Spawning environment '" + self.scenario + "'")

        environment = CollisionObject()
        environment.id = self.scenario
        environment.header.stamp = rospy.Time.now()
        environment.header.frame_id = "base_link"
        filename = rospkg.RosPack().get_path("cob_grasping") + self.data[self.scenario]["environment"]["mesh"]
        scale = self.data[self.scenario]["environment"]["scaling"]
        environment.meshes.append(self.load_mesh(filename, scale))
        q = quaternion_from_euler(self.data[self.scenario]["environment"]["orientation"][0]/180.0 * math.pi,
                                  self.data[self.scenario]["environment"]["orientation"][1]/180.0 * math.pi,
                                  self.data[self.scenario]["environment"]["orientation"][2]/180.0 * math.pi)
        position = [self.data[self.scenario]["environment"]["position"][0],
                    self.data[self.scenario]["environment"]["position"][1],
                    self.data[self.scenario]["environment"]["position"][2],
                    q[0], q[1], q[2], q[3]]
        add_remove_object("add", copy(environment), position, "mesh")

        if len(self.data[self.scenario]["environment"]["additional_obstacles"]) != 0 and self.spawn_obstacles != "none":
            for item in self.data[self.scenario]["environment"]["additional_obstacles"]:
                if self.spawn_obstacles == "all":
                    collision_object = CollisionObject()
                    collision_object.header.stamp = rospy.Time.now()
                    collision_object.header.frame_id = "base_link"
                    collision_object.id = item["id"]
                    object_shape = SolidPrimitive()
                    object_shape.type = object_shape.BOX
                    object_shape.dimensions.append(item["size"][0])  # X
                    object_shape.dimensions.append(item["size"][1])  # Y
                    object_shape.dimensions.append(item["size"][2])  # Z
                    collision_object.primitives.append(object_shape)
                    q = quaternion_from_euler(item["orientation"][0]/180.0 * math.pi,
                                              item["orientation"][1]/180.0 * math.pi,
                                              item["orientation"][2]/180.0 * math.pi)
                    position = [item["position"][0], item["position"][1], item["position"][2], q[0], q[1], q[2], q[3]]
                    add_remove_object("add", collision_object, position, "primitive")
                elif item["id"] == self.spawn_obstacles:
                    collision_object = CollisionObject()
                    collision_object.header.stamp = rospy.Time.now()
                    collision_object.header.frame_id = "base_link"
                    collision_object.id = item["id"]
                    object_shape = SolidPrimitive()
                    object_shape.type = object_shape.BOX
                    object_shape.dimensions.append(item["size"][0])  # X
                    object_shape.dimensions.append(item["size"][1])  # Y
                    object_shape.dimensions.append(item["size"][2])  # Z
                    collision_object.primitives.append(object_shape)
                    q = quaternion_from_euler(item["orientation"][0]/180.0 * math.pi,
                                              item["orientation"][1]/180.0 * math.pi,
                                              item["orientation"][2]/180.0 * math.pi)
                    position = [item["position"][0], item["position"][1], item["position"][2], q[0], q[1], q[2], q[3]]
                    add_remove_object("add", collision_object, position, "primitive")
                    break

        self.spawn_marker()

    def clear_environment(self):
        co_object = CollisionObject()
        for i in self.data:
            co_object.id = i
            add_remove_object("remove", copy(co_object), "", "")
            if len(self.data[i]["environment"]["additional_obstacles"]) != 0:
                for item in self.data[i]["environment"]["additional_obstacles"]:
                    co_object.id = item["id"]
                    add_remove_object("remove", copy(co_object), "", "")

        co_object.id = "object"
        add_remove_object("remove", copy(co_object), "", "")

        self.server.clear()
        self.server.applyChanges()

    def spawn_marker(self):
        color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        # ---- BUILD POSITION MARKER ----
        self.make_marker("right_arm_start", copy(color), InteractiveMarkerControl.MOVE_3D,
                         Point(self.data[self.scenario]["positions"]["start_r"][0],
                               self.data[self.scenario]["positions"]["start_r"][1],
                               self.data[self.scenario]["positions"]["start_r"][2]))
        self.make_marker("right_arm_goal", copy(color), InteractiveMarkerControl.MOVE_3D,
                         Point(self.data[self.scenario]["positions"]["goal_r"][0],
                               self.data[self.scenario]["positions"]["goal_r"][1],
                               self.data[self.scenario]["positions"]["goal_r"][2]))
        self.make_marker("left_arm_goal", copy(color), InteractiveMarkerControl.MOVE_3D,
                         Point(self.data[self.scenario]["positions"]["goal_l"][0],
                               self.data[self.scenario]["positions"]["goal_l"][1],
                               self.data[self.scenario]["positions"]["goal_l"][2]))

        # ---- BUILD WAYPOINT MARKER ----
        if self.use_waypoints:
            color.r = 0.0
            color.b = 1.0
            if len(self.data[self.scenario]["positions"]["waypoints_r"]) != 0:
                for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_r"])):
                    self.make_marker("waypoint_r" + str(i + 1), color, InteractiveMarkerControl.MOVE_3D,
                                     Point(self.data[self.scenario]["positions"]["waypoints_r"][i][0],
                                           self.data[self.scenario]["positions"]["waypoints_r"][i][1],
                                           self.data[self.scenario]["positions"]["waypoints_r"][i][2]))

            if len(self.data[self.scenario]["positions"]["waypoints_l"]) != 0:
                for i in xrange(0, len(self.data[self.scenario]["positions"]["waypoints_l"])):
                    self.make_marker("waypoint_l" + str(i + 1), color, InteractiveMarkerControl.MOVE_3D,
                                     Point(self.data[self.scenario]["positions"]["waypoints_l"][i][0],
                                           self.data[self.scenario]["positions"]["waypoints_l"][i][1],
                                           self.data[self.scenario]["positions"]["waypoints_l"][i][2]))

            self.server.applyChanges()

        # ---- APPLY MENU TO MARKER ----
        self.menu_handler.apply(self.server, "right_arm_start")
        self.menu_handler.apply(self.server, "right_arm_goal")
        self.menu_handler.apply(self.server, "left_arm_goal")

        self.server.applyChanges()


class StartPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_max', 'error_counter', 'joint_trajectory_speed'],
                             output_keys=['error_message', 'error_counter'])

        self.traj_name = "pre_grasp"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right

        global abort_execution

        if abort_execution:
            abort_execution = False
            userdata.error_message = "Execution aborted by user"
            return "error"

        try:
            traj = plan_movement(self.planer, userdata.active_arm, self.traj_name, userdata.joint_trajectory_speed)
        except (ValueError, IndexError):
            if userdata.error_counter >= userdata.error_max:
                userdata.error_counter = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm \
                                         + " arm"
                return "error"

            userdata.error_counter += 1
            return "failed"
        else:
            self.planer.execute(traj)
            return "succeeded"


class EndPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_max', 'arm_positions', 'object', 'error_counter',
                                         'planning_method', 'joint_trajectory_speed'],
                             output_keys=['error_message', 'error_counter'])

        self.traj_name = "retreat"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right

        if userdata.planning_method != "joint":
            # ----------- SPAWN OBJECT ------------
            collision_object = CollisionObject()
            collision_object.header.stamp = rospy.Time.now()
            collision_object.header.frame_id = "base_link"
            collision_object.id = "object"
            object_shape = SolidPrimitive()
            object_shape.type = object_shape.CYLINDER
            object_shape.dimensions.append(userdata.object[2])  # Height
            object_shape.dimensions.append(userdata.object[0]*0.5)  # Radius
            collision_object.primitives.append(object_shape)
            add_remove_object("remove", copy(collision_object), "", "")
            position = [userdata.arm_positions[userdata.active_arm]["goal"].x,
                        userdata.arm_positions[userdata.active_arm]["goal"].y,
                        userdata.arm_positions[userdata.active_arm]["goal"].z,
                        0.0, 0.0, 0.0, 1.0]
            add_remove_object("add", copy(collision_object), position, "primitive")

        global abort_execution

        if abort_execution:
            abort_execution = False
            userdata.error_message = "Execution aborted by user"
            return "error"

        try:
            traj = plan_movement(self.planer, userdata.active_arm, self.traj_name, userdata.joint_trajectory_speed)
        except (ValueError, IndexError):
            if userdata.error_counter >= userdata.error_max:
                userdata.error_counter = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm\
                                         + " arm"
                return "error"

            userdata.error_counter += 1
            return "failed"
        else:
            self.planer.execute(traj)

            if userdata.planning_method != "joint":
                # ----------- REMOVE OBJECT ------------
                add_remove_object("remove", collision_object, "", "")
            return "succeeded"


class PlanningAndExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'cs_orientation', 'error_max', 'error_message', 'object',
                                         'manipulation_options', 'arm_positions', 'error_counter', 'planning_method',
                                         'joint_trajectory_speed'],
                             output_keys=['cs_position', 'cs_orientation', 'error_message',
                                          'error_counter'])

        self.tf_listener = tf.TransformListener()

        self.eef_step = rospy.get_param(str(rospy.get_name()) + "/eef_step")
        self.jump_threshold = rospy.get_param(str(rospy.get_name()) + "/jump_threshold")
        self.planer_id = rospy.get_param(str(rospy.get_name()) + "/planer_id")

        rospy.loginfo("Set planer to: '" + str(self.planer_id) + "'")

        self.traj_name = ""
        self.traj_pick = []
        self.traj_place = []
        self.plan_list = []

        self.joint_names = {"right": rospy.get_param("/arm_right/joint_names"),
                            "left": rospy.get_param("/arm_left/joint_names")}

        self.cs_ready = False
        self.last_state = 0

    def execute(self, userdata):

        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        self.planer.set_planner_id(self.planer_id)

        if userdata.cs_orientation[2] >= 0.5 * math.pi:
            # Rotate clockwise
            userdata.cs_orientation[3] = -1.0
        elif userdata.cs_orientation[2] <= -0.5 * math.pi:
            # Rotate counterclockwise
            userdata.cs_orientation[3] = 1.0

        if userdata.cs_orientation[3] == 1.0:
            userdata.cs_orientation[2] += 5.0 / 180.0 * math.pi
        elif userdata.cs_orientation[3] == -1.0:
            userdata.cs_orientation[2] -= 5.0 / 180.0 * math.pi

        if userdata.active_arm == "left":
            userdata.cs_orientation[0] = math.pi
        elif userdata.active_arm == "right":
            userdata.cs_orientation[0] = 0.0

        global abort_execution

        if abort_execution:
            abort_execution = False
            userdata.error_message = "Execution aborted by user"
            userdata.cs_position = "start"
            return "error"

        if userdata.planning_method != "joint":
            execution = self.plan_and_move_cartesian(userdata)
        else:
            execution = self.plan_and_move_joint(userdata)

        if not execution:
            if userdata.error_counter >= userdata.error_max:
                userdata.error_counter = 0
                self.traj_pick[:] = []
                self.traj_place[:] = []
                userdata.cs_position = "start"
                userdata.cs_orientation[2] = 0.0
                return "error"
            else:
                return "failed"

        userdata.error_counter = 0
        return "succeeded"

    def plan_and_move_cartesian(self, userdata):

        if len(self.traj_pick) < 3:

            # -------------------- PICK --------------------
            # ----------- APPROACH -----------
            self.traj_name = "approach"
            traj_approach = RobotTrajectory()
            start_state = RobotState()

            start_state.joint_state.name = self.joint_names[userdata.active_arm]
            start_state.joint_state.position = self.planer.get_current_joint_values()
            start_state.attached_collision_objects[:] = []
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            approach_pose_offset = PoseStamped()
            approach_pose_offset.header.frame_id = "current_object"
            approach_pose_offset.header.stamp = rospy.Time(0)
            approach_pose_offset.pose.position.x = -userdata.manipulation_options["approach_dist"]
            approach_pose_offset.pose.orientation.w = 1

            try:
                approach_pose = self.tf_listener.transformPose("base_link", approach_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_pick[:] = []
                return False

            if userdata.planning_method == "cartesian_linear":

                (traj_approach, frac_approach) = self.planer.compute_cartesian_path([approach_pose.pose], self.eef_step,
                                                                                    self.jump_threshold, True)

                if frac_approach < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")
                elif 0.5 <= frac_approach < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")

                if frac_approach != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False

            elif userdata.planning_method == "cartesian" or userdata.planning_method == "cartesian_mixed":

                self.planer.clear_pose_targets()
                self.planer.set_pose_target(approach_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_approach = self.planer.plan()
                    traj_approach = smooth_cartesian_path(traj_approach)
                    traj_approach = scale_joint_trajectory_speed(traj_approach, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_pick.append(traj_approach)

            # ----------- GRASP -----------
            self.traj_name = "grasp"
            traj_grasp = RobotTrajectory()
            start_state = RobotState()

            traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
            start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
            start_state.joint_state.position = traj_approach_endpoint.positions
            start_state.attached_collision_objects[:] = []
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            grasp_pose_offset = PoseStamped()
            grasp_pose_offset.header.frame_id = "current_object"
            grasp_pose_offset.header.stamp = rospy.Time(0)
            grasp_pose_offset.pose.orientation.w = 1

            try:
                grasp_pose = self.tf_listener.transformPose("base_link", grasp_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_pick[:] = []
                return False

            if userdata.planning_method == "cartesian_linear" or userdata.planning_method == "cartesian_mixed":
                (traj_grasp, frac_grasp) = self.planer.compute_cartesian_path([grasp_pose.pose], self.eef_step,
                                                                              self.jump_threshold, True)

                if frac_grasp < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")
                elif 0.5 <= frac_grasp < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")

                if frac_grasp != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False

            elif userdata.planning_method == "cartesian":

                self.planer.clear_pose_targets()
                self.planer.set_pose_target(grasp_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_grasp = self.planer.plan()
                    traj_grasp = smooth_cartesian_path(traj_grasp)
                    traj_grasp = scale_joint_trajectory_speed(traj_grasp, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_pick.append(traj_grasp)

            # ----------- LIFT -----------
            self.traj_name = "lift"
            traj_lift = RobotTrajectory()
            start_state = RobotState()

            traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
            start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
            start_state.joint_state.position = traj_grasp_endpoint.positions

            # Attach object
            object_shape = SolidPrimitive()
            object_shape.type = object_shape.CYLINDER
            object_shape.dimensions.append(userdata.object[2])  # Height
            object_shape.dimensions.append(userdata.object[0]*0.5)  # Radius

            object_pose = Pose()
            object_pose.orientation.w = 1.0

            object_collision = CollisionObject()
            object_collision.header.frame_id = "gripper_" + userdata.active_arm + "_grasp_link"
            object_collision.id = "object"
            object_collision.primitives.append(object_shape)
            object_collision.primitive_poses.append(object_pose)
            object_collision.operation = CollisionObject.ADD

            object_attached = AttachedCollisionObject()
            object_attached.link_name = "gripper_" + userdata.active_arm + "_grasp_link"
            object_attached.object = object_collision
            object_attached.touch_links = ["gripper_" + userdata.active_arm + "_base_link",
                                           "gripper_" + userdata.active_arm + "_camera_link",
                                           "gripper_" + userdata.active_arm + "_finger_1_link",
                                           "gripper_" + userdata.active_arm + "_finger_2_link",
                                           "gripper_" + userdata.active_arm + "_grasp_link",
                                           "gripper_" + userdata.active_arm + "_palm_link"]

            start_state.attached_collision_objects.append(object_attached)

            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            lift_pose_offset = PoseStamped()
            lift_pose_offset.header.frame_id = "current_object"
            lift_pose_offset.header.stamp = rospy.Time(0)
            if userdata.active_arm == "left":
                lift_pose_offset.pose.position.z = -userdata.manipulation_options["lift_height"]
            elif userdata.active_arm == "right":
                lift_pose_offset.pose.position.z = userdata.manipulation_options["lift_height"]
            lift_pose_offset.pose.orientation.w = 1

            try:
                lift_pose = self.tf_listener.transformPose("base_link", lift_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_pick[:] = []
                return False

            if userdata.planning_method == "cartesian_linear" or userdata.planning_method == "cartesian_mixed":

                (traj_lift, frac_lift) = self.planer.compute_cartesian_path([lift_pose.pose], self.eef_step,
                                                                            self.jump_threshold, True)

                if frac_lift < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")
                elif 0.5 <= frac_lift < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")

                if frac_lift != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False

            elif userdata.planning_method == "cartesian":

                self.planer.clear_pose_targets()
                self.planer.set_pose_target(lift_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_lift = self.planer.plan()
                    traj_lift = smooth_cartesian_path(traj_lift)
                    traj_lift = scale_joint_trajectory_speed(traj_lift, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_pick[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_pick.append(traj_lift)

            userdata.cs_position = "goal"
            userdata.error_counter = 0
            userdata.cs_orientation[2] = 0.0

            rospy.loginfo("Pick planning complete")
            return False

        else:

            if not self.cs_ready:
                rospy.logwarn("Target coordinate system not ready")
                self.cs_ready = True
                rospy.sleep(0.05)
                return False

            # -------------------- PLACE --------------------
            # ----------- MOVE -----------
            self.traj_name = "move"
            traj_move = RobotTrajectory()

            try:
                traj_lift_endpoint = self.traj_pick[2].joint_trajectory.points[-1]
            except AttributeError:
                self.traj_pick[:] = []
                self.traj_place[:] = []
                userdata.cs_position = "start"
                userdata.error_message = "Error: " + str(AttributeError)
                userdata.error_counter += 1
                return False

            start_state = RobotState()
            start_state.joint_state.name = self.traj_pick[2].joint_trajectory.joint_names
            start_state.joint_state.position = traj_lift_endpoint.positions

            # Attach object
            object_shape = SolidPrimitive()
            object_shape.type = object_shape.CYLINDER
            object_shape.dimensions.append(userdata.object[2])  # Height
            object_shape.dimensions.append(userdata.object[0]*0.5)  # Radius

            object_pose = Pose()
            object_pose.orientation.w = 1.0

            object_collision = CollisionObject()
            object_collision.header.frame_id = "gripper_" + userdata.active_arm + "_grasp_link"
            object_collision.id = "object"
            object_collision.primitives.append(object_shape)
            object_collision.primitive_poses.append(object_pose)
            object_collision.operation = CollisionObject.ADD

            object_attached = AttachedCollisionObject()
            object_attached.link_name = "gripper_" + userdata.active_arm + "_grasp_link"
            object_attached.object = object_collision
            object_attached.touch_links = ["gripper_" + userdata.active_arm + "_base_link",
                                           "gripper_" + userdata.active_arm + "_camera_link",
                                           "gripper_" + userdata.active_arm + "_finger_1_link",
                                           "gripper_" + userdata.active_arm + "_finger_2_link",
                                           "gripper_" + userdata.active_arm + "_grasp_link",
                                           "gripper_" + userdata.active_arm + "_palm_link"]

            start_state.attached_collision_objects.append(object_attached)

            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            self.planer.clear_pose_targets()
            move_pose_offset = PoseStamped()
            move_pose_offset.header.frame_id = "current_object"
            move_pose_offset.header.stamp = rospy.Time(0)
            if userdata.active_arm == "left":
                move_pose_offset.pose.position.z = -userdata.manipulation_options["lift_height"]
            elif userdata.active_arm == "right":
                move_pose_offset.pose.position.z = userdata.manipulation_options["lift_height"]
            move_pose_offset.pose.orientation.w = 1

            try:
                move_pose = self.tf_listener.transformPose("base_link", move_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_place[:] = []
                return False

            if userdata.planning_method == "cartesian_linear":

                way_move = []

                if len(userdata.arm_positions[userdata.active_arm]["waypoints"]) != 0:
                    for item in userdata.arm_positions[userdata.active_arm]["waypoints"]:
                        wpose_offset = PoseStamped()
                        wpose_offset.header.frame_id = "current_object"
                        wpose_offset.header.stamp = rospy.Time(0)
                        wpose_offset.pose.orientation.w = 1

                        try:
                            wpose = self.tf_listener.transformPose("base_link", wpose_offset)
                        except Exception, e:
                            userdata.error_message = "Could not transform pose. Exception: " + str(e)
                            userdata.error_counter += 1
                            self.traj_place[:] = []
                            return False

                        wpose.pose.position = item
                        way_move.append(wpose.pose)

                way_move.append(move_pose.pose)

                (traj_move, frac_move) = self.planer.compute_cartesian_path(way_move, self.eef_step,
                                                                            self.jump_threshold, True)

                if frac_move < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")
                elif 0.5 <= frac_move < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")

                if frac_move != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False

            elif userdata.planning_method == "cartesian" or userdata.planning_method == "cartesian_mixed":
                self.planer.clear_pose_targets()
                self.planer.set_pose_target(move_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_move = self.planer.plan()
                    traj_move = smooth_cartesian_path(traj_move)
                    traj_move = scale_joint_trajectory_speed(traj_move, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_place.append(traj_move)

            # ----------- DROP -----------
            self.traj_name = "drop"
            traj_drop = RobotTrajectory()
            start_state = RobotState()

            traj_move_endpoint = traj_move.joint_trajectory.points[-1]
            start_state.joint_state.name = traj_move.joint_trajectory.joint_names
            start_state.joint_state.position = traj_move_endpoint.positions

            # Attach object
            object_shape = SolidPrimitive()
            object_shape.type = object_shape.CYLINDER
            object_shape.dimensions.append(userdata.object[2])  # Height
            object_shape.dimensions.append(userdata.object[0]*0.5)  # Radius

            object_pose = Pose()
            object_pose.orientation.w = 1.0

            object_collision = CollisionObject()
            object_collision.header.frame_id = "gripper_" + userdata.active_arm + "_grasp_link"
            object_collision.id = "object"
            object_collision.primitives.append(object_shape)
            object_collision.primitive_poses.append(object_pose)
            object_collision.operation = CollisionObject.ADD

            object_attached = AttachedCollisionObject()
            object_attached.link_name = "gripper_" + userdata.active_arm + "_grasp_link"
            object_attached.object = object_collision
            object_attached.touch_links = ["gripper_" + userdata.active_arm + "_base_link",
                                           "gripper_" + userdata.active_arm + "_camera_link",
                                           "gripper_" + userdata.active_arm + "_finger_1_link",
                                           "gripper_" + userdata.active_arm + "_finger_2_link",
                                           "gripper_" + userdata.active_arm + "_grasp_link",
                                           "gripper_" + userdata.active_arm + "_palm_link"]

            start_state.attached_collision_objects.append(object_attached)

            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            drop_pose_offset = PoseStamped()
            drop_pose_offset.header.frame_id = "current_object"
            drop_pose_offset.pose.orientation.w = 1
            try:
                drop_pose = self.tf_listener.transformPose("base_link", drop_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_place[:] = []
                return False

            if userdata.planning_method == "cartesian_linear" or userdata.planning_method == "cartesian_mixed":

                (traj_drop, frac_drop) = self.planer.compute_cartesian_path([drop_pose.pose], self.eef_step,
                                                                            self.jump_threshold, True)

                if frac_drop < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")
                elif 0.5 <= frac_drop < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")

                if frac_drop != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False

            elif userdata.planning_method == "cartesian":

                self.planer.clear_pose_targets()
                self.planer.set_pose_target(drop_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_drop = self.planer.plan()
                    traj_drop = smooth_cartesian_path(traj_drop)
                    traj_drop = scale_joint_trajectory_speed(traj_drop, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_place.append(traj_drop)

            # ----------- RETREAT -----------
            self.traj_name = "retreat"
            traj_retreat = RobotTrajectory()
            start_state = RobotState()

            traj_drop_endpoint = traj_drop.joint_trajectory.points[-1]
            start_state.joint_state.name = traj_drop.joint_trajectory.joint_names
            start_state.joint_state.position = traj_drop_endpoint.positions
            start_state.attached_collision_objects[:] = []
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            retreat_pose_offset = PoseStamped()
            retreat_pose_offset.header.frame_id = "current_object"
            retreat_pose_offset.header.stamp = rospy.Time(0)
            retreat_pose_offset.pose.position.x = -userdata.manipulation_options["approach_dist"]
            retreat_pose_offset.pose.orientation.w = 1

            try:
                retreat_pose = self.tf_listener.transformPose("base_link", retreat_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                userdata.error_counter += 1
                self.traj_place[:] = []
                return False

            if userdata.planning_method == "cartesian_linear" or userdata.planning_method == "cartesian_mixed":

                (traj_retreat, frac_retreat) = self.planer.compute_cartesian_path([retreat_pose.pose], self.eef_step,
                                                                                  self.jump_threshold, True)

                if frac_retreat < 0.5:
                    rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")
                elif 0.5 <= frac_retreat < 1.0:
                    rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")

                if frac_retreat != 1.0:
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False

            elif userdata.planning_method == "cartesian":

                self.planer.clear_pose_targets()
                self.planer.set_pose_target(retreat_pose.pose, self.planer.get_end_effector_link())

                try:
                    traj_retreat = self.planer.plan()
                    traj_retreat = smooth_cartesian_path(traj_retreat)
                    traj_retreat = scale_joint_trajectory_speed(traj_retreat, userdata.joint_trajectory_speed)
                except (ValueError, IndexError):
                    rospy.loginfo("Plan " + self.traj_name + ": failed")
                    userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                    userdata.error_counter += 1
                    self.traj_place[:] = []
                    return False
                else:
                    rospy.loginfo("Plan " + self.traj_name + ": succeeded")

            self.traj_place.append(traj_retreat)

            rospy.loginfo("Place planning complete")

        # ----------- TRAJECTORY OPERATIONS -----------
        computed_trajectories = self.traj_pick + self.traj_place

        rospy.loginfo("Smooth trajectories and fix velocities")
        try:
            for i in xrange(0, len(computed_trajectories)):
                computed_trajectories[i] = smooth_cartesian_path(computed_trajectories[i])
                computed_trajectories[i] = fix_velocities(computed_trajectories[i])
        except (ValueError, IndexError, AttributeError):
            computed_trajectories[:] = []
            self.traj_pick[:] = []
            self.traj_place[:] = []
            userdata.cs_position = "start"
            userdata.error_message = "Error: " + str(AttributeError)
            userdata.error_counter += 1
            return False

        # ----------- EXECUTE -----------
        rospy.loginfo("---- Start execution ----")
        rospy.loginfo("------- Approach -------")
        self.planer.execute(computed_trajectories[0])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("-------- Grasp --------")
        self.planer.execute(computed_trajectories[1])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("-------- Lift --------")
        self.planer.execute(computed_trajectories[2])
        rospy.loginfo("-------- Move --------")
        self.planer.execute(computed_trajectories[3])
        rospy.loginfo("-------- Drop --------")
        self.planer.execute(computed_trajectories[4])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("------- Retreat -------")
        self.planer.execute(computed_trajectories[5])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("- Execution finished -")

        # ----------- CLEAR TRAJECTORY LIST -----------
        computed_trajectories[:] = []
        self.traj_pick[:] = []
        self.traj_place[:] = []

        userdata.cs_position = "start"
        self.cs_ready = False

        return True

    def plan_and_move_joint(self, userdata):

        for i in xrange(self.last_state, len(userdata.arm_positions[userdata.active_arm]["joints"])):
            start_state = RobotState()
            start_state.joint_state.name = self.joint_names[userdata.active_arm]

            if i == 0:
                start_state.joint_state.position = self.planer.get_current_joint_values()
            else:
                start_state.joint_state.position = self.plan_list[i - 1].joint_trajectory.points[-1].positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            self.planer.clear_pose_targets()
            self.planer.set_joint_value_target(userdata.arm_positions[userdata.active_arm]["joints"][i])

            try:
                plan = self.planer.plan()
                plan = smooth_cartesian_path(plan)
                plan = scale_joint_trajectory_speed(plan, userdata.joint_trajectory_speed)
            except (ValueError, IndexError, AttributeError):
                rospy.logerr("Planning trajectory " + str(i + 1) + " failed")
                userdata.error_message = "Error: " + str(AttributeError)
                userdata.error_counter += 1
                self.last_state = i
                return False

            else:
                rospy.loginfo("Planned trajectory " + str(i + 1) + " successfully")
                self.plan_list.append(plan)

        # ----------- EXECUTE -----------
        rospy.loginfo("---- Start execution ----")
        rospy.loginfo("------- Approach -------")
        self.planer.execute(self.plan_list[0])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("-------- Grasp --------")
        self.planer.execute(self.plan_list[1])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("-------- Lift --------")
        self.planer.execute(self.plan_list[2])
        rospy.loginfo("-------- Move --------")
        self.planer.execute(self.plan_list[3])
        rospy.loginfo("-------- Drop --------")
        self.planer.execute(self.plan_list[4])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("------- Retreat -------")
        self.planer.execute(self.plan_list[5])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("- Execution finished -")

        # ----------- CLEAR TRAJECTORY LIST -----------
        self.plan_list[:] = []
        self.last_state = 0

        return True

    @staticmethod
    def move_gripper(component_name, pos):
        error_code = -1
        counter = 0
        while not rospy.is_shutdown() and error_code != 0:
            rospy.loginfo("Trying to move " + str(component_name) + " to " + str(pos) + " retries: " + str(counter))
            handle = sss.move(component_name, pos)
            handle.wait()
            error_code = handle.get_error_code()
            if counter > 100:
                rospy.logerr(component_name + "does not work any more. Retries: " + str(counter) +
                             ". Please reset USB connection and press <ENTER>.")
                sss.wait_for_input()
                return False
            counter += 1
        return True


class SwitchArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'finished', 'switch_targets'],
                             input_keys=['active_arm', 'cs_orientation', 'arm_positions', 'manipulation_options',
                                         'switch_arm'],
                             output_keys=['active_arm', 'cs_orientation', 'arm_positions', 'cs_position'])

        self.counter = 1

    def execute(self, userdata):
        if self.counter == userdata.manipulation_options["repeats"]:
            return "finished"

        elif not userdata.switch_arm:
            userdata.cs_position = "start"
            userdata.cs_orientation[2] = 0.0
            self.counter += 1.0
            return "switch_targets"

        elif userdata.switch_arm:
            if self.counter % 2 == 0:
                userdata.cs_position = "start"
                userdata.cs_orientation[2] = 0.0
                self.counter += 1.0
                return "switch_targets"

            if userdata.active_arm == "left":
                userdata.active_arm = "right"
                userdata.cs_orientation[3] = 1.0
                userdata.cs_orientation[0] = 0.0

            elif userdata.active_arm == "right":
                userdata.active_arm = "left"
                userdata.cs_orientation[3] = -1.0
                userdata.cs_orientation[0] = math.pi

            userdata.cs_position = "start"
            userdata.cs_orientation[2] = 0.0
            self.counter += 1.0
            return "succeeded"


class SwitchTargets(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['arm_positions', 'active_arm', 'planning_method'],
                             output_keys=['arm_positions'])

    def execute(self, userdata):
        if userdata.planning_method != "joint":
            (userdata.arm_positions["right"]["start"], userdata.arm_positions["right"]["goal"]) =\
                self.switch_values(userdata.arm_positions["right"]["start"],
                                   userdata.arm_positions["right"]["goal"])
            (userdata.arm_positions["left"]["start"], userdata.arm_positions["left"]["goal"]) =\
                self.switch_values(userdata.arm_positions["left"]["start"],
                                   userdata.arm_positions["left"]["goal"])

            userdata.arm_positions["right"]["waypoints"] = \
                list(reversed(sorted(userdata.arm_positions["right"]["waypoints"])))
            userdata.arm_positions["left"]["waypoints"] = \
                list(reversed(sorted(userdata.arm_positions["left"]["waypoints"])))
        else:
            userdata.arm_positions["right"]["joints"] = \
                list(reversed(userdata.arm_positions["right"]["joints"]))
            userdata.arm_positions["left"]["joints"] = \
                list(reversed(userdata.arm_positions["left"]["joints"]))
        return "succeeded"

    @staticmethod
    def switch_values(item1, item2):
        temp = item1
        item1 = item2
        item2 = temp
        return item1, item2


class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished'],
                             input_keys=['error_message'])

    def execute(self, userdata):
        rospy.logerr(userdata.error_message)
        return "finished"


class SM(smach.StateMachine):
    def __init__(self):

        smach.StateMachine.__init__(self, outcomes=['ended'])

        self.userdata.active_arm = rospy.get_param(str(rospy.get_name()) + "/arm")
        self.userdata.planning_method = rospy.get_param(str(rospy.get_name()) + "/planning_method")
        self.userdata.joint_trajectory_speed = rospy.get_param(str(rospy.get_name()) + "/joint_trajectory_speed")
        self.userdata.switch_arm = bool
        self.userdata.cs_position = "start"

        self.userdata.error_max = rospy.get_param(str(rospy.get_name()) + "/max_error")
        self.userdata.manipulation_options = {"lift_height": rospy.get_param(str(rospy.get_name()) + "/lift_height"),
                                              "approach_dist": rospy.get_param(str(rospy.get_name())
                                                                               + "/approach_distance"),
                                              "repeats": rospy.get_param(str(rospy.get_name())
                                                                         + "/manipulation_repeats")}

        self.userdata.cs_orientation = [0.0,  # Roll (x)
                                        0.0,  # Pitch (y)
                                        0.0,  # Yaw (z)
                                        1.0]  # Direction for rotation

        # ---- ARM-POSITIONS ----
        self.userdata.arm_positions = {"right": {"start": Point(),
                                                 "waypoints": [],
                                                 "goal": Point()},
                                       "left": {"start": Point(),
                                                "waypoints": [],
                                                "goal": Point()}}

        # ---- ERROR MESSAGE / COUNTER ----
        self.userdata.error_message = ""
        self.userdata.error_counter = 0

        # ---- OBJECT DIMENSIONS ----
        self.userdata.object = [float]*3  # Diameter in x, diameter in y, height

        if self.userdata.planning_method != "joint":
            # ---- TF BROADCASTER ----
            self.tf_listener = tf.TransformListener()
            self.br = tf.TransformBroadcaster()
            rospy.Timer(rospy.Duration.from_sec(0.01), self.broadcast_tf)
        '''
        # ---- DYNAMIC RECONFIGURE SERVER ---
        Server(parameterConfig, self.dynrec_callback)
        '''

        with self:

            smach.StateMachine.add('SCENE_MANAGER', SceneManager(),
                                   transitions={'succeeded': 'START_POSITION',
                                                'exit': 'ended'})

            smach.StateMachine.add('START_POSITION', StartPosition(),
                                   transitions={'succeeded': 'PLANNINGANDEXECUTION',
                                                'failed': 'START_POSITION',
                                                'error': 'ERROR'})

            smach.StateMachine.add('PLANNINGANDEXECUTION', PlanningAndExecution(),
                                   transitions={'succeeded': 'END_POSITION',
                                                'failed': 'PLANNINGANDEXECUTION',
                                                'error': 'ERROR'})

            smach.StateMachine.add('END_POSITION', EndPosition(),
                                   transitions={'succeeded': 'SWITCH_ARM',
                                                'failed': 'END_POSITION',
                                                'error': 'ERROR'})

            smach.StateMachine.add('SWITCH_ARM', SwitchArm(),
                                   transitions={'succeeded': 'START_POSITION',
                                                'switch_targets': 'SWITCH_TARGETS',
                                                'finished': 'ended'})

            smach.StateMachine.add('SWITCH_TARGETS', SwitchTargets(),
                                   transitions={'succeeded': 'START_POSITION'})

            smach.StateMachine.add('ERROR', Error(),
                                   transitions={'finished': 'SCENE_MANAGER'})

    def dynrec_callback(self, config, level):
        self.userdata.error_max = config["error_max"]
        self.userdata.manipulation_options = {"lift_height": config["lift_height"],
                                              "approach_dist": config["approach_dist"],
                                              "repeats": config["manipulation_repeats"]}
        rospy.loginfo("Dynamic reconfigure: 'error_max': " + str(self.userdata.error_max) + " | 'lift_height': "
                      + str(self.userdata.manipulation_options["lift_height"]) + " | 'approach_dist': "
                      + str(self.userdata.manipulation_options["approach_dist"]) + " | 'manipulation_repeats': "
                      + str(self.userdata.manipulation_options["repeats"]))

        return config

    def broadcast_tf(self, event):
        self.br.sendTransform(
            (self.userdata.arm_positions[self.userdata.active_arm][self.userdata.cs_position].x,
             self.userdata.arm_positions[self.userdata.active_arm][self.userdata.cs_position].y,
             self.userdata.arm_positions[self.userdata.active_arm][self.userdata.cs_position].z),
            quaternion_from_euler(self.userdata.cs_orientation[0], self.userdata.cs_orientation[1],
                                  self.userdata.cs_orientation[2]),
            event.current_real,
            "current_object",
            "base_link")


if __name__ == '__main__':
    rospy.init_node('grasping_test')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    # rospy.spin()
    sis.stop()
