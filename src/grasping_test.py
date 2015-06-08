#!/usr/bin/python
import rospkg
import smach
import smach_ros
import tf
import yaml
from pyassimp import pyassimp
from copy import copy

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


def plan_movement(planer, arm, pose):
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
        plan = scale_joint_trajectory_speed(plan, 0.3)
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


class SetTargets(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['active_arm', 'arm_positions', 'environment'],
                             output_keys=['arm_positions'])

        self.scenario = "table"

        rospy.loginfo("Reading positions from yaml file...")
        self.path = rospkg.RosPack().get_path("cob_grasping") + "/config/positions.yaml"
        data = self.load_positions(self.path)

        self.start_r = data[0]
        self.goal_r = data[1]
        self.goal_l = data[2]
        self.waypoints_r = data[3]
        self.waypoints_l = data[4]

        self.server = InteractiveMarkerServer("grasping_targets")
        self.menu_handler = MenuHandler()

        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0

        self.make_marker("right_arm_start", copy(color), InteractiveMarkerControl.MOVE_3D, self.start_r)
        self.make_marker("right_arm_goal", copy(color), InteractiveMarkerControl.MOVE_3D, self.goal_r)
        self.make_marker("left_arm_goal", copy(color), InteractiveMarkerControl.MOVE_3D, self.goal_l)

        color.r = 0.0
        color.b = 1.0

        self.make_marker("waypoint_r1", color, InteractiveMarkerControl.MOVE_3D, self.waypoints_r[0])
        self.make_marker("waypoint_r2", color, InteractiveMarkerControl.MOVE_3D, self.waypoints_r[1])
        self.make_marker("waypoint_l1", color, InteractiveMarkerControl.MOVE_3D, self.waypoints_l[0])
        self.make_marker("waypoint_l2", color, InteractiveMarkerControl.MOVE_3D, self.waypoints_l[1])

        self.server.applyChanges()

        self.menu_handler.insert("Start execution", callback=self.start_planning)
        self.menu_handler.insert("Stopp execution", callback=self.stop_planning)
        h_entry = self.menu_handler.insert("Environment")

        self.ids = []
        self.environment = {}

        for x in id_list:
            self.ids.append(x)

        self.menu_handler.setCheckState(self.menu_handler.insert(env_list[0].title(), callback=self.change_environment,
                                                                 parent=h_entry), MenuHandler.CHECKED)
        self.menu_handler.setCheckState(self.menu_handler.insert(env_list[1].title(), callback=self.change_environment,
                                                                 parent=h_entry), MenuHandler.UNCHECKED)
        self.menu_handler.setCheckState(self.menu_handler.insert(env_list[2].title(), callback=self.change_environment,
                                                                 parent=h_entry), MenuHandler.UNCHECKED)

        self.menu_handler.apply(self.server, "right_arm_start")
        self.menu_handler.apply(self.server, "right_arm_goal")
        self.menu_handler.apply(self.server, "left_arm_goal")
        self.menu_handler.apply(self.server, "waypoint_r1")
        self.menu_handler.apply(self.server, "waypoint_r2")
        self.menu_handler.apply(self.server, "waypoint_l1")
        self.menu_handler.apply(self.server, "waypoint_l2")

        self.server.applyChanges()

        self.start_manipulation = threading.Event()

    def execute(self, userdata):
        self.environment = userdata.environment
        self.spawn_environment()
        rospy.loginfo("Click start to continue...")
        self.start_manipulation.wait()
        self.start_manipulation.clear()

        # Position for right arm
        userdata.arm_positions["right"] = {"start": self.start_r,
                                           "waypoints": self.waypoints_r,
                                           "goal": self.goal_r}

        # Position for left arm
        userdata.arm_positions["left"] = {"start": self.goal_r,
                                          "waypoints": self.waypoints_l,
                                          "goal": self.goal_l}

        rospy.loginfo("Writing positions to yaml file...")
        self.save_positions(self.path)

        return "succeeded"

    def load_positions(self, filename):
        values = [Point(), Point(), Point(), [Point(), Point()], [Point(), Point()]]
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)

        values[0] = Point(doc[self.scenario]["start_r"][0],
                          doc[self.scenario]["start_r"][1],
                          doc[self.scenario]["start_r"][2])
        values[1] = Point(doc[self.scenario]["goal_r"][0],
                          doc[self.scenario]["goal_r"][1],
                          doc[self.scenario]["goal_r"][2])
        values[2] = Point(doc[self.scenario]["goal_l"][0],
                          doc[self.scenario]["goal_l"][1],
                          doc[self.scenario]["goal_l"][2])
        values[3] = [Point(doc[self.scenario]["waypoints_r"][0][0],
                           doc[self.scenario]["waypoints_r"][0][1],
                           doc[self.scenario]["waypoints_r"][0][2]),
                     Point(doc[self.scenario]["waypoints_r"][1][0],
                           doc[self.scenario]["waypoints_r"][1][1],
                           doc[self.scenario]["waypoints_r"][1][2])]
        values[4] = [Point(doc[self.scenario]["waypoints_l"][0][0],
                           doc[self.scenario]["waypoints_l"][0][1],
                           doc[self.scenario]["waypoints_l"][0][2]),
                     Point(doc[self.scenario]["waypoints_l"][1][0],
                           doc[self.scenario]["waypoints_l"][1][1],
                           doc[self.scenario]["waypoints_l"][1][2])]

        return values

    def save_positions(self, filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        self.set_in_dict(doc, [self.scenario, "start_r"], [round(self.start_r.x, 3),
                                                           round(self.start_r.y, 3),
                                                           round(self.start_r.z, 3)])

        self.set_in_dict(doc, [self.scenario, "goal_r"], [round(self.goal_r.x, 3),
                                                          round(self.goal_r.y, 3),
                                                          round(self.goal_r.z, 3)])

        self.set_in_dict(doc, [self.scenario, "goal_l"], [round(self.goal_l.x, 3),
                                                          round(self.goal_l.y, 3),
                                                          round(self.goal_l.z, 3)])

        self.set_in_dict(doc, [self.scenario, "waypoints_r"], [[round(self.waypoints_r[0].x, 3),
                                                                round(self.waypoints_r[0].y, 3),
                                                                round(self.waypoints_r[0].z, 3)],
                                                               [round(self.waypoints_r[1].x, 3),
                                                                round(self.waypoints_r[1].y, 3),
                                                                round(self.waypoints_r[1].z, 3)]])

        self.set_in_dict(doc, [self.scenario, "waypoints_l"], [[round(self.waypoints_l[0].x, 3),
                                                                round(self.waypoints_l[0].y, 3),
                                                                round(self.waypoints_l[0].z, 3)],
                                                               [round(self.waypoints_l[1].x, 3),
                                                                round(self.waypoints_l[1].y, 3),
                                                                round(self.waypoints_l[1].z, 3)]])
        stream = file(filename, 'w')
        yaml.dump(doc, stream)

    @staticmethod
    def get_from_dict(datadict, maplist):
        return reduce(lambda d, k: d[k], maplist, datadict)

    def set_in_dict(self, datadict, maplist, value):
        self.get_from_dict(datadict, maplist[:-1])[maplist[-1]] = value

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

    @staticmethod
    def make_box(color):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale.x = object_dim[0]  # diameter in x
        marker.scale.y = object_dim[1]  # diameter in y
        marker.scale.z = object_dim[2]  # height
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
                self.start_r = feedback.pose.position
            elif feedback.marker_name == "right_arm_goal":
                self.goal_r = feedback.pose.position
            elif feedback.marker_name == "left_arm_goal":
                self.goal_l = feedback.pose.position
            elif feedback.marker_name == "waypoint_r1":
                self.waypoints_r[0] = feedback.pose.position
            elif feedback.marker_name == "waypoint_r2":
                self.waypoints_r[1] = feedback.pose.position
            elif feedback.marker_name == "waypoint_l1":
                self.waypoints_l[0] = feedback.pose.position
            elif feedback.marker_name == "waypoint_l2":
                self.waypoints_l[1] = feedback.pose.position

            rospy.loginfo("Position " + feedback.marker_name + ": x = " + str(feedback.pose.position.x)
                          + " | y = " + str(feedback.pose.position.y) + " | z = " + str(feedback.pose.position.z))
            self.server.applyChanges()

    def start_planning(self, feedback):
        self.start_manipulation.set()

    @staticmethod
    def stop_planning(feedback):
        error_counter[0] = 999
        error_counter[1] = 999

    def change_environment(self, feedback):
        if self.menu_handler.getCheckState(feedback.menu_entry_id) == MenuHandler.UNCHECKED:
            for env_id in self.ids:
                if env_id == feedback.menu_entry_id:
                    self.menu_handler.setCheckState(env_id, MenuHandler.CHECKED)
                else:
                    self.menu_handler.setCheckState(env_id, MenuHandler.UNCHECKED)
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            # Set new scenario
            self.scenario = id_list[feedback.menu_entry_id]

            # Spawn new environment
            self.spawn_environment()

            # Load positions
            rospy.loginfo("Reading positions from yaml file...")
            data = self.load_positions(self.path)

            self.start_r = data[0]
            self.goal_r = data[1]
            self.goal_l = data[2]
            self.waypoints_r = data[3]
            self.waypoints_l = data[4]

            position = Pose()

            position.position = self.start_r
            self.server.setPose("right_arm_start", copy(position))
            position.position = self.goal_r
            self.server.setPose("right_arm_goal", copy(position))
            position.position = self.goal_l
            self.server.setPose("left_arm_goal", copy(position))
            position.position = self.waypoints_r[0]
            self.server.setPose("waypoint_r1", copy(position))
            position.position = self.waypoints_r[1]
            self.server.setPose("waypoint_r2", copy(position))
            position.position = self.waypoints_l[0]
            self.server.setPose("waypoint_l1", copy(position))
            position.position = self.waypoints_l[1]
            self.server.setPose("waypoint_l2", copy(position))

            self.server.applyChanges()

    def spawn_environment(self):
        # Clear environment
        rospy.loginfo("Spawning environment '" + self.scenario + "'")
        self.clear_environment()

        environment = CollisionObject()
        environment.id = self.scenario
        environment.header.stamp = rospy.Time.now()
        environment.header.frame_id = "odom_combined"
        filename = self.environment[self.scenario]["path"]
        scale = self.environment[self.scenario]["scale"]
        environment.meshes.append(self.load_mesh(filename, scale))
        add_remove_object("add", copy(environment), self.environment[self.scenario]["position"], "mesh")

        if "add_objects" in self.environment[self.scenario]:
            for i in xrange(0, len(self.environment[self.scenario]["add_objects"]), 1):

                collision_object = CollisionObject()
                collision_object.header.stamp = rospy.Time.now()
                collision_object.header.frame_id = "odom_combined"
                collision_object.id = self.environment[self.scenario]["add_objects"][i]["id"]
                object_shape = SolidPrimitive()
                object_shape.type = object_shape.BOX
                object_shape.dimensions.append(self.environment[self.scenario]["add_objects"][i]["size"][0])  # X
                object_shape.dimensions.append(self.environment[self.scenario]["add_objects"][i]["size"][1])  # Y
                object_shape.dimensions.append(self.environment[self.scenario]["add_objects"][i]["size"][2])  # Z
                collision_object.primitives.append(object_shape)
                add_remove_object("add", collision_object,
                                  self.environment[self.scenario]["add_objects"][i]["position"], "primitive")

    def clear_environment(self):
        co_object = CollisionObject()
        for i in self.environment:
            co_object.id = i
            add_remove_object("remove", copy(co_object), "", "")
            if "add_objects" in self.environment[i]:
                for x in xrange(0, len(self.environment[i]["add_objects"]), 1):
                    co_object.id = self.environment[i]["add_objects"][x]["id"]
                    add_remove_object("remove", copy(co_object), "", "")

        co_object.id = "object"
        add_remove_object("remove", copy(co_object), "", "")


class StartPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_max'],
                             output_keys=['error_message'])

        self.traj_name = "pre_grasp"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return "failed"

        try:
            traj = plan_movement(self.planer, userdata.active_arm, self.traj_name)
        except (ValueError, IndexError):
            if error_counter[0] >= userdata.error_max[0]:
                error_counter[0] = 0
                error_counter[1] = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm\
                                         + " arm"
                return "error"

            error_counter[0] += 1
            return "failed"
        else:
            self.planer.execute(traj)
            return "succeeded"


class EndPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_max', 'arm_positions', 'object'],
                             output_keys=['error_message'])

        self.traj_name = "retreat"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return "error"

        # ----------- SPAWN OBJECT ------------
        collision_object = CollisionObject()
        collision_object.header.stamp = rospy.Time.now()
        collision_object.header.frame_id = "odom_combined"
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

        try:
            traj = plan_movement(self.planer, userdata.active_arm, self.traj_name)
        except (ValueError, IndexError):
            if error_counter[0] >= userdata.error_max[0]:
                error_counter[0] = 0
                error_counter[1] = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm\
                                         + " arm"
                return "error"

            error_counter[0] += 1
            return "failed"
        else:
            self.planer.execute(traj)

            # ----------- REMOVE OBJECT ------------
            add_remove_object("remove", collision_object, "", "")
            return "succeeded"


class PlanningAndExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'cs_orientation', 'computed_trajectories', 'error_max',
                                         'error_message', 'object', 'manipulation_options', 'arm_positions'],
                             output_keys=['cs_position', 'cs_orientation', 'computed_trajectories', 'error_message'])

        self.tf_listener = tf.TransformListener()

        self.eef_step = 0.01
        self.jump_threshold = 2

        self.traj_name = ""

    def execute(self, userdata):

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

        if not self.plan_and_move(userdata):
            if error_counter[0] >= userdata.error_max[0]:
                error_counter[0] = 0
                error_counter[1] = 0
                userdata.computed_trajectories[:] = []
                userdata.computed_trajectories = [False]*6
                userdata.cs_position = "start"
                return "error"
            elif error_counter[1] >= userdata.error_max[1]:
                error_counter[0] = 0
                error_counter[1] = 0
                userdata.computed_trajectories[:] = []
                userdata.computed_trajectories = [False]*6
                userdata.cs_position = "start"
                return "error"
            else:
                return "failed"

        error_counter[0] = 0
        error_counter[1] = 0
        return "succeeded"

    def plan_and_move(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return "error"

        # ----------- LOAD CONFIG -----------
        (config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm, "pre_grasp")
        if error_code != 0:
            userdata.error_message = "Unable to parse pre_grasp configuration"
            return "error"

        if not (userdata.computed_trajectories[0] and userdata.computed_trajectories[1]
                and userdata.computed_trajectories[2]):

            # -------------------- PICK --------------------
            # ----------- APPROACH -----------
            self.traj_name = "approach"
            start_state = RobotState()
            start_state.joint_state.name = config.joint_names
            start_state.joint_state.position = config.points[0].positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            approach_pose_offset = PoseStamped()
            approach_pose_offset.header.frame_id = "current_object"
            approach_pose_offset.header.stamp = rospy.Time(0)
            approach_pose_offset.pose.position.x = -userdata.manipulation_options["approach_dist"]
            approach_pose_offset.pose.orientation.w = 1
            try:
                approach_pose = self.tf_listener.transformPose("odom_combined", approach_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_listener.clear()
                print str(e)
                error_counter[1] += 1
                return False

            (traj_approach, frac_approach) = self.planer.compute_cartesian_path([approach_pose.pose], self.eef_step,
                                                                                self.jump_threshold, True)

            if frac_approach < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")
            elif 0.5 <= frac_approach < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")

            if not (frac_approach == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[0] = traj_approach

            # ----------- GRASP -----------
            self.traj_name = "grasp"
            traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
            start_state.joint_state.position = traj_approach_endpoint.positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            grasp_pose_offset = PoseStamped()
            grasp_pose_offset.header.frame_id = "current_object"
            grasp_pose_offset.header.stamp = rospy.Time(0)
            grasp_pose_offset.pose.orientation.w = 1

            try:
                grasp_pose = self.tf_listener.transformPose("odom_combined", grasp_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_listener.clear()
                print str(e)
                error_counter[1] += 1
                return False

            (traj_grasp, frac_grasp) = self.planer.compute_cartesian_path([grasp_pose.pose], self.eef_step,
                                                                          self.jump_threshold, True)

            if frac_grasp < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")
            elif 0.5 <= frac_grasp < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")

            if not (frac_grasp == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[1] = traj_grasp

            # ----------- LIFT -----------
            self.traj_name = "lift"
            traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
            start_state = RobotState()
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
                lift_pose = self.tf_listener.transformPose("odom_combined", lift_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_listener.clear()
                print str(e)
                error_counter[1] += 1
                return False

            (traj_lift, frac_lift) = self.planer.compute_cartesian_path([lift_pose.pose], self.eef_step,
                                                                        self.jump_threshold, True)

            if frac_lift < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")
            elif 0.5 <= frac_lift < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")

            if not (frac_lift == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[2] = traj_lift

            userdata.cs_position = "goal"
            error_counter[0] = 0
            error_counter[1] = 0
            rospy.loginfo("Pick planning complete")
            return False

        else:
            # -------------------- PLACE --------------------
            # ----------- MOVE -----------
            self.traj_name = "move"
            try:
                traj_lift_endpoint = userdata.computed_trajectories[2].joint_trajectory.points[-1]
            except AttributeError:
                userdata.computed_trajectories[:] = []
                userdata.computed_trajectories = [False]*6
                userdata.error_message = "Error: " + str(AttributeError)
                error_counter[0] += 1
                return False

            start_state = RobotState()
            start_state.joint_state.name = userdata.computed_trajectories[2].joint_trajectory.joint_names
            start_state.joint_state.position = traj_lift_endpoint.positions
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
                move_pose = self.tf_listener.transformPose("odom_combined", move_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                error_counter[1] += 1
                return False

            way_move = []

            for i in xrange(0, len(userdata.arm_positions[userdata.active_arm]["waypoints"])):
                wpose_offset = PoseStamped()
                wpose_offset.header.frame_id = "current_object"
                wpose_offset.header.stamp = rospy.Time(0)
                wpose_offset.pose.orientation.w = 1

                try:
                    wpose = self.tf_listener.transformPose("odom_combined", wpose_offset)
                except Exception, e:
                    userdata.error_message = "Could not transform pose. Exception: " + str(e)
                    error_counter[1] += 1
                    return False

                wpose.pose.position = userdata.arm_positions[userdata.active_arm]["waypoints"][i]
                way_move.append(wpose.pose)

            way_move.append(move_pose.pose)

            (traj_move, frac_move) = self.planer.compute_cartesian_path(way_move, self.eef_step, self.jump_threshold,
                                                                        True)

            if frac_move < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")
            elif 0.5 <= frac_move < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")

            if not (frac_move == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            if len(traj_move.joint_trajectory.points) < 15:
                rospy.logerr("Computed trajectory is too short. Replanning...")
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[3] = traj_move

            # ----------- DROP -----------
            self.traj_name = "drop"
            traj_move_endpoint = traj_move.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_move.joint_trajectory.joint_names
            start_state.joint_state.position = traj_move_endpoint.positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            drop_pose_offset = PoseStamped()
            drop_pose_offset.header.frame_id = "current_object"
            drop_pose_offset.pose.orientation.w = 1
            try:
                drop_pose = self.tf_listener.transformPose("odom_combined", drop_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                error_counter[1] += 1
                return False

            (traj_drop, frac_drop) = self.planer.compute_cartesian_path([drop_pose.pose], self.eef_step,
                                                                        self.jump_threshold, True)

            if frac_drop < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")
            elif 0.5 <= frac_drop < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")

            if not (frac_drop == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[4] = traj_drop

            # ----------- RETREAT -----------
            self.traj_name = "retreat"
            traj_drop_endpoint = traj_drop.joint_trajectory.points[-1]
            start_state = RobotState()
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
                retreat_pose = self.tf_listener.transformPose("odom_combined", retreat_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                error_counter[1] += 1
                return False

            (traj_retreat, frac_retreat) = self.planer.compute_cartesian_path([retreat_pose.pose], self.eef_step,
                                                                              self.jump_threshold, True)

            if frac_retreat < 0.5:
                rospy.logerr("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")
            elif 0.5 <= frac_retreat < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")

            if not (frac_retreat == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                error_counter[0] += 1
                return False

            userdata.computed_trajectories[5] = traj_retreat

            rospy.loginfo("Place planning complete")

        # ----------- TRAJECTORY OPERATIONS -----------
        rospy.loginfo("Smooth trajectories")
        try:
            userdata.computed_trajectories[0] = smooth_cartesian_path(userdata.computed_trajectories[0])
            userdata.computed_trajectories[1] = smooth_cartesian_path(userdata.computed_trajectories[1])
            userdata.computed_trajectories[2] = smooth_cartesian_path(userdata.computed_trajectories[2])
            userdata.computed_trajectories[3] = smooth_cartesian_path(userdata.computed_trajectories[3])
            userdata.computed_trajectories[4] = smooth_cartesian_path(userdata.computed_trajectories[4])
            userdata.computed_trajectories[5] = smooth_cartesian_path(userdata.computed_trajectories[5])
        except (ValueError, IndexError, AttributeError):
            userdata.computed_trajectories[:] = []
            userdata.computed_trajectories = [False]*6
            userdata.error_message = "Error: " + str(AttributeError)
            error_counter[0] += 1
            return False

        rospy.loginfo("Fix velocities")
        userdata.computed_trajectories[0] = fix_velocities(userdata.computed_trajectories[0])
        userdata.computed_trajectories[1] = fix_velocities(userdata.computed_trajectories[1])
        userdata.computed_trajectories[2] = fix_velocities(userdata.computed_trajectories[2])
        userdata.computed_trajectories[3] = fix_velocities(userdata.computed_trajectories[3])
        userdata.computed_trajectories[4] = fix_velocities(userdata.computed_trajectories[4])
        userdata.computed_trajectories[5] = fix_velocities(userdata.computed_trajectories[5])

        # ----------- EXECUTE -----------
        rospy.loginfo("---- Start execution ----")
        rospy.loginfo("Approach")
        self.planer.execute(userdata.computed_trajectories[0])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("Grasp")
        self.planer.execute(userdata.computed_trajectories[1])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("Lift")
        self.planer.execute(userdata.computed_trajectories[2])
        rospy.loginfo("Move")
        self.planer.execute(userdata.computed_trajectories[3])
        rospy.loginfo("Drop")
        self.planer.execute(userdata.computed_trajectories[4])
        # self.move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("Retreat")
        self.planer.execute(userdata.computed_trajectories[5])
        # self.move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("---- Execution finished ----")

        # ----------- CLEAR TRAJECTORY LIST -----------
        userdata.computed_trajectories[:] = []
        userdata.computed_trajectories = [False]*6

        userdata.cs_position = "start"

        return True

    @staticmethod
    def move_gripper(component_name, pos):
        error_code = -1
        counter = 0
        while not rospy.is_shutdown() and error_code != 0:
            print "Trying to move", component_name, "to", pos, "retries: ", counter
            handle = sss.move(component_name, pos)
            handle.wait()
            error_code = handle.get_error_code()
            if counter > 100:
                rospy.logerr(component_name + "does not work any more. retries: " + str(counter) +
                             ". Please reset USB connection and press <ENTER>.")
                sss.wait_for_input()
                return False
            counter += 1
        return True


class SwitchArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'finished', 'switch_targets'],
                             input_keys=['active_arm', 'cs_orientation', 'arm_positions', 'manipulation_options'],
                             output_keys=['active_arm', 'cs_orientation', 'arm_positions', 'cs_position'])

        self.counter = 1

    def execute(self, userdata):
        if self.counter == userdata.manipulation_options["repeats"]:
            return "finished"
        else:
            if self.counter % 2 == 0:
                userdata.cs_position = "start"
                userdata.cs_orientation[2] = 0.0
                self.counter += 1.0
                return "switch_targets"

            if userdata.active_arm == "left":
                userdata.active_arm = "right"
                userdata.cs_orientation[3] = 1.0
            elif userdata.active_arm == "right":
                userdata.active_arm = "left"
                userdata.cs_orientation[3] = -1.0

            userdata.cs_position = "start"
            userdata.cs_orientation[2] = 0.0
            self.counter += 1.0
            return "succeeded"


class SwitchTargets(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['arm_positions', 'active_arm'],
                             output_keys=['arm_positions'])

    def execute(self, userdata):
        (userdata.arm_positions["right"]["start"], userdata.arm_positions["right"]["goal"]) =\
            self.switch_values(userdata.arm_positions["right"]["start"],
                               userdata.arm_positions["right"]["goal"])
        (userdata.arm_positions["left"]["start"], userdata.arm_positions["left"]["goal"]) =\
            self.switch_values(userdata.arm_positions["left"]["start"],
                               userdata.arm_positions["left"]["goal"])

        userdata.arm_positions["right"]["waypoints"] = list(reversed(userdata.arm_positions["right"]["waypoints"]))
        userdata.arm_positions["left"]["waypoints"] = list(reversed(userdata.arm_positions["left"]["waypoints"]))
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

        global object_dim, env_list, id_list, error_counter
        object_dim = [0.02, 0.02, 0.1]

        self.userdata.active_arm = "right"
        self.userdata.cs_position = "start"
        self.userdata.manipulation_options = {"lift_height": float,
                                              "approach_dist": float,
                                              "repeats": int}

        self.userdata.cs_orientation = [0.0,  # roll (x)
                                        0.0,  # pitch (y)
                                        (0.0 / 180.0) * math.pi,  # yaw (z)
                                        1.0]  # direction for rotation

        # ---- ARM-POSITIONS ----
        self.userdata.arm_positions = {"right": {"start": Point(),
                                                 "waypoints": [0.0]*2,
                                                 "goal": Point()},
                                       "left": {"start": Point(),
                                                "waypoints": [0.0]*2,
                                                "goal": Point()}}

        # ---- TRAJECTORY LIST ----
        self.userdata.computed_trajectories = [False]*6

        # ---- ERROR MESSAGE / COUNTER ----
        self.userdata.error_message = ""
        error_counter = [0]*2

        # ---- OBJECT DIMENSIONS ----
        self.userdata.object = [object_dim[0], object_dim[1], object_dim[2]]  # diameter in x, diameter in y, height

        # ---- INIT ENVIRONMENT ----
        env_list = ["table", "rack", "shelf"]
        id_list = {4: "table", 5: "rack", 6: "shelf"}

        # --- PATH ---
        path = [rospkg.RosPack().get_path("cob_grasping") + "/files/table.stl",
                rospkg.RosPack().get_path("cob_grasping") + "/files/rack.stl",
                rospkg.RosPack().get_path("cob_grasping") + "/files/shelf_unit.stl"]

        # --- SCALE ---
        scale = [1.0, 0.002, 0.01]

        # --- POSITION ---
        position = [[0.0]*7]*3
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        position[0] = [0.56, 0.0, 0.61, q[0], q[1], q[2], q[3]]

        q = quaternion_from_euler(0.5*math.pi, 0.0, 0.5*math.pi)
        position[1] = [0.56, -0.64, 0.0, q[0], q[1], q[2], q[3]]
        position[2] = [0.67, -0.12, 0.16, q[0], q[1], q[2], q[3]]

        # --- ENVIRONMENT ---
        self.userdata.environment = {}

        # -- ADDITIONAL OBJECTS --
        add_objects = [{"id": "wall_r",
                        "size": [0.48, 0.05, 0.5],
                        "position": [0.8, -0.15, 0.86, 0.0, 0.0, 0.0, 1.0]},
                       {"id": "wall_l",
                        "size": [0.48, 0.05, 0.5],
                        "position": [0.8, 0.15, 0.86, 0.0, 0.0, 0.0, 1.0]}]

        for i in xrange(0, len(env_list), 1):
            if env_list[i] == "table":
                self.userdata.environment[env_list[i]] = {"path": path[i],
                                                          "scale": scale[i],
                                                          "position": position[i],
                                                          "add_objects": add_objects}
            else:
                self.userdata.environment[env_list[i]] = {"path": path[i],
                                                          "scale": scale[i],
                                                          "position": position[i]}

        # ---- TF BROADCASTER ----
        self.tf_listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration.from_sec(0.01), self.broadcast_tf)

        # ---- DYNAMIC RECONFIGURE SERVER ---
        Server(parameterConfig, self.dynreccallback)

        with self:

            smach.StateMachine.add('SET_TARGETS', SetTargets(),
                                   transitions={'succeeded': 'START_POSITION'})

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
                                   transitions={'finished': 'SET_TARGETS'})

    def dynreccallback(self, config, level):
        self.userdata.error_max = [config["error_plan_max"], config["error_tf_max"]]
        self.userdata.manipulation_options = {"lift_height": config["lift_height"],
                                              "approach_dist": config["approach_dist"],
                                              "repeats": config["manipulation_repeats"]}
        rospy.loginfo("Reconfigure: " + str(self.userdata.error_max[0]) + " | " + str(self.userdata.error_max[1])
                      + " | " + str(self.userdata.manipulation_options["lift_height"]) + " | "
                      + str(self.userdata.manipulation_options["approach_dist"]) + " | "
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
