#!/usr/bin/python
import rospkg
import smach
import smach_ros
import tf

from pyassimp import pyassimp

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.msg import RobotTrajectory
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive

from simple_script_server import *
sss = simple_script_server()
mgc_left = MoveGroupCommander("arm_left")
mgc_right = MoveGroupCommander("arm_right")

planning_scene = PlanningScene()
planning_scene.is_diff = True

planning_scene_interface = PlanningSceneInterface()

active_arm = "right"


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


def smooth_cartesian_path(traj):

    time_offset = 200000000  # 0.2s

    for i in range(len(traj.joint_trajectory.points)):
        traj.joint_trajectory.points[i].time_from_start += rospy.Duration(0, time_offset)

    traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(0, time_offset)

    return traj


def fix_velocities(traj):
    # fix trajectories to stop at the end
    traj.joint_trajectory.points[-1].velocities = [0]*7

    # fix trajectories to be slower
    speed_factor = 1.0
    for i in range(len(traj.joint_trajectory.points)):
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
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        points[i] = point

    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj


class SpawnObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['target', 'active_arm'])
        self.pub_planning_scene = rospy.Publisher("planning_scene", PlanningScene, queue_size=1)

    def execute(self, userdata):
        # Initialize objects
        rospy.loginfo("Add table to world")
        table = CollisionObject()
        table.id = "table"
        table.header.stamp = rospy.Time.now()
        table.header.frame_id = "odom_combined"
        filename = rospkg.RosPack().get_path("cob_grasping") + "/files/table.stl"
        table.meshes.append(self.load_mesh(filename))
        self.add_remove_object("remove", table, "", "")
        position = [0.37, -0.3, 0.62, 1.0]
        self.add_remove_object("add", table, position, "mesh")

        rospy.loginfo("Add an object to world")
        collision_object = CollisionObject()
        collision_object.header.stamp = rospy.Time.now()
        collision_object.header.frame_id = "odom_combined"
        collision_object.id = "object"
        object_shape = SolidPrimitive()
        object_shape.type = 3  # Cylinder
        object_shape.dimensions.append(0.2)  # Height
        object_shape.dimensions.append(0.01)  # Radius
        collision_object.primitives.append(object_shape)
        self.add_remove_object("remove", collision_object, "", "")
        position = [userdata.target[0][0], userdata.target[0][1] + 0.02, userdata.target[0][2], 1.0]
        self.add_remove_object("add", collision_object, position, "primitive")

        return 'succeeded'

    def add_remove_object(self, co_operation, co_object, co_position, co_type):
        if co_operation == "add":
            co_object.operation = CollisionObject.ADD
            pose = Pose()
            pose.position.x = co_position[0]
            pose.position.y = co_position[1]
            pose.position.z = co_position[2]
            pose.orientation.w = co_position[3]
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
        self.pub_planning_scene.publish(planning_scene)
        rospy.sleep(1)

    @staticmethod
    def load_mesh(filename):

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
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            mesh.vertices.append(point)
        pyassimp.release(scene)
        return mesh


class StartPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_plan_max'],
                             output_keys=['error_message'])

        self.planning_error = 0
        self.traj_name = "pre_grasp"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return 'failed'

        try:
            traj = self.plan_movement(userdata.active_arm, self.traj_name)
        except (ValueError, IndexError):
            if self.planning_error == userdata.error_plan_max:
                self.planning_error = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm\
                                         + " arm"
                return "error"

            self.planning_error += 1
            return "failed"
        else:
            self.planer.execute(traj)
            return "succeeded"

    def plan_movement(self, arm, pose):
        (config, error_code) = sss.compose_trajectory("arm_" + arm, pose)
        if error_code != 0:
            rospy.logerr("Unable to parse " + pose + " configuration")

        start_state = RobotState()
        start_state.joint_state.name = config.joint_names

        start_state.joint_state.position = self.planer.get_current_joint_values()
        self.planer.set_start_state(start_state)

        self.planer.clear_pose_targets()
        self.planer.set_joint_value_target(config.points[0].positions)

        plan = self.planer.plan()

        plan = smooth_cartesian_path(plan)
        plan = scale_joint_trajectory_speed(plan, 0.3)
        return plan


class EndPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'error_plan_max'],
                             output_keys=['error_message'])

        self.planning_error = 0
        self.traj_name = "retreat"

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return "error"

        try:
            traj = self.plan_movement(userdata.active_arm, self.traj_name)
        except (ValueError, IndexError):
            if self.planning_error == userdata.error_plan_max:
                self.planning_error = 0
                userdata.error_message = "Unabled to plan " + self.traj_name + " trajectory for " + userdata.active_arm\
                                         + " arm"
                return "error"

            self.planning_error += 1
            return "failed"
        else:
            self.planer.execute(traj)
            return "succeeded"

    def plan_movement(self, arm, pose):
        (config, error_code) = sss.compose_trajectory("arm_" + arm, pose)
        if error_code != 0:
            rospy.logerr("Unable to parse " + pose + " configuration")

        start_state = RobotState()
        start_state.joint_state.name = config.joint_names

        start_state.joint_state.position = self.planer.get_current_joint_values()
        self.planer.set_start_state(start_state)

        self.planer.clear_pose_targets()
        self.planer.set_joint_value_target(config.points[0].positions)

        plan = self.planer.plan()

        plan = smooth_cartesian_path(plan)
        plan = scale_joint_trajectory_speed(plan, 0.3)
        return plan


class Manipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'error'],
                             input_keys=['active_arm', 'cs_data', 'target_cs', 'target', 'trajectories', 'error_tf_max', 'error_plan_max', 'error_message'],
                             output_keys=['target_cs', 'cs_data', 'trajectories', 'error_message'])
        self.angle_offset_yaw = 0.0
        self.angle_offset_pitch = 0.0
        self.cs_position_x = 0.0
        self.cs_position_y = 0.0
        self.cs_position_z = 0.0

        if active_arm == "left":
            self.angle_offset_roll = math.pi
        elif active_arm == "right":
            self.angle_offset_roll = 0.0

        self.eef_step = 0.01
        self.jump_threshold = 2

        self.tf_listener = tf.TransformListener()

        rospy.Timer(rospy.Duration.from_sec(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()

        self.planning_error = 0
        self.tf_error = 0
        self.traj_name = ""

    def broadcast_tf(self, event):
        self.br.sendTransform(
            (self.cs_position_x, self.cs_position_y, self.cs_position_z),
            quaternion_from_euler(self.angle_offset_roll, self.angle_offset_pitch, self.angle_offset_yaw),
            event.current_real,
            "current_object",
            "base_link")

    def execute(self, userdata):
        self.cs_position_x = userdata.target[userdata.target_cs][0]
        self.cs_position_y = userdata.target[userdata.target_cs][1]
        self.cs_position_z = userdata.target[userdata.target_cs][2]

        if userdata.cs_data[2] >= 0.5 * math.pi:
            userdata.cs_data[3] = -1.0
        elif userdata.cs_data[2] <= -0.5 * math.pi:
            userdata.cs_data[3] = 1.0

        if userdata.cs_data[3] == 1.0:
            userdata.cs_data[2] += 5.0 / 180.0 * math.pi
        elif userdata.cs_data[3] == -1.0:
            userdata.cs_data[2] -= 5.0 / 180.0 * math.pi

        if userdata.active_arm == "left":
            userdata.cs_data[0] = math.pi
        elif userdata.active_arm == "right":
            userdata.cs_data[0] = 0

        self.angle_offset_roll = userdata.cs_data[0]
        self.angle_offset_pitch = userdata.cs_data[1]
        self.angle_offset_yaw = userdata.cs_data[2]

        if not self.plan_and_move(userdata):
            if self.planning_error == userdata.error_plan_max:
                self.planning_error = 0
                return "error"
            elif self.tf_error == userdata.error_tf_max:
                self.tf_error = 0
                return "error"

            return "failed"

        self.planning_error = 0
        return "succeeded"

    def plan_and_move(self, userdata):
        if userdata.active_arm == "left":
            self.planer = mgc_left
        elif userdata.active_arm == "right":
            self.planer = mgc_right
        else:
            userdata.error_message = "Invalid arm"
            return "error"

        # Plan Pick-Trajectorie
        if userdata.trajectories[0] == 0.0 or userdata.trajectories[1] == 0.0 or userdata.trajectories[2] == 0.0:

            # -------------------- PICK --------------------
            # ----------- APPROACH -----------
            self.traj_name = "approach"
            start_state = RobotState()
            (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm, "pre_grasp")
            if error_code != 0:
                userdata.error_message = "Unable to parse pre_grasp configuration"
                self.planning_error += 1
                return False

            start_state.joint_state.name = pre_grasp_config.joint_names
            start_state.joint_state.position = pre_grasp_config.points[0].positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            approach_pose_offset = PoseStamped()
            approach_pose_offset.header.frame_id = "current_object"
            approach_pose_offset.header.stamp = rospy.Time(0)
            approach_pose_offset.pose.position.x = -0.12
            approach_pose_offset.pose.orientation.w = 1
            try:
                approach_pose = self.tf_listener.transformPose("odom_combined", approach_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_error += 1
                return False

            (traj_approach, frac_approach) = self.planer.compute_cartesian_path([approach_pose.pose],
                                                                                self.eef_step,
                                                                                self.jump_threshold,
                                                                                True)

            if frac_approach < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_approach * 100, 2)) + "%")

            if not (frac_approach == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                self.planning_error += 1
                return False

            userdata.trajectories[0] = traj_approach

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
            grasp_pose = self.tf_listener.transformPose("odom_combined", grasp_pose_offset)
            (traj_grasp, frac_grasp) = self.planer.compute_cartesian_path([grasp_pose.pose],
                                                                          self.eef_step, self.jump_threshold,
                                                                          True)

            if frac_grasp < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_grasp * 100, 2)) + "%")

            if not (frac_grasp == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                self.planning_error += 1
                return False

            userdata.trajectories[1] = traj_grasp

            collision_object = CollisionObject()
            collision_object.header.stamp = rospy.Time.now()
            collision_object.header.frame_id = "odom_combined"
            collision_object.id = "object"
            object_shape = SolidPrimitive()
            object_shape.type = 3  # Cylinder
            object_shape.dimensions.append(0.2)  # Height
            object_shape.dimensions.append(0.01)  # Radius
            collision_object.primitives.append(object_shape)
            # SpawnObjects.add_remove_object(SpawnObjects, "remove", collision_object, "", "")

            # ----------- LIFT -----------
            self.traj_name = "lift"
            traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
            start_state.joint_state.position = traj_grasp_endpoint.positions

            # Attach object
            object_shape = SolidPrimitive()
            object_shape.type = 3  # CYLINDER
            object_shape.dimensions.append(0.2)
            object_shape.dimensions.append(0.01)

            object_pose = Pose()
            object_pose.orientation.w = 1.0

            object_collision = CollisionObject()
            object_collision.header.frame_id = "gripper_"+userdata.active_arm+"_grasp_link"
            object_collision.id = "object"
            object_collision.primitives.append(object_shape)
            object_collision.primitive_poses.append(object_pose)
            object_collision.operation = CollisionObject.ADD

            object_attached = AttachedCollisionObject()
            object_attached.link_name = "gripper_"+userdata.active_arm+"_grasp_link"
            object_attached.object = object_collision
            object_attached.touch_links = ["gripper_"+userdata.active_arm+"_base_link",
                                           "gripper_"+userdata.active_arm+"_camera_link",
                                           "gripper_"+userdata.active_arm+"_finger_1_link",
                                           "gripper_"+userdata.active_arm+"_finger_2_link",
                                           "gripper_"+userdata.active_arm+"_grasp_link",
                                           "gripper_"+userdata.active_arm+"_palm_link"]

            # start_state.attached_collision_objects.append(object_attached)

            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            lift_pose_offset = PoseStamped()
            lift_pose_offset.header.frame_id = "current_object"
            lift_pose_offset.header.stamp = rospy.Time(0)
            if userdata.active_arm == "left":
                lift_pose_offset.pose.position.z = -0.2
            elif userdata.active_arm == "right":
                lift_pose_offset.pose.position.z = 0.2
            lift_pose_offset.pose.orientation.w = 1
            lift_pose = self.tf_listener.transformPose("odom_combined", lift_pose_offset)

            (traj_lift, frac_lift) = self.planer.compute_cartesian_path([lift_pose.pose],
                                                                        self.eef_step, self.jump_threshold,
                                                                        True)

            if frac_lift < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_lift * 100, 2)) + "%")

            if not (frac_lift == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                self.planning_error += 1
                return False

            userdata.trajectories[2] = traj_lift

            userdata.target_cs = 1
            rospy.loginfo("Pick planning complete")
            return False

        # Plan Place-Trajectorie
        else:

            # -------------------- PLACE --------------------
            # ----------- MOVE -----------
            self.traj_name = "move"
            traj_lift_endpoint = userdata.trajectories[2].joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = userdata.trajectories[2].joint_trajectory.joint_names
            start_state.joint_state.position = traj_lift_endpoint.positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            move_pose_offset = PoseStamped()
            move_pose_offset.header.frame_id = "current_object"
            move_pose_offset.header.stamp = rospy.Time(0)
            if userdata.active_arm == "left":
                move_pose_offset.pose.position.z = -0.2
            elif userdata.active_arm == "right":
                move_pose_offset.pose.position.z = 0.2
            move_pose_offset.pose.orientation.w = 1
            try:
                move_pose = self.tf_listener.transformPose("odom_combined", move_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_error += 1
                return False

            (traj_move, frac_move) = self.planer.compute_cartesian_path([move_pose.pose],
                                                                        self.eef_step, self.jump_threshold,
                                                                        True)

            if frac_move < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_move * 100, 2)) + "%")

            if not (frac_move == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                return False

            userdata.trajectories[3] = traj_move

            if len(traj_move.joint_trajectory.points) < 10:
                rospy.logerr("Computed trajectory is too short. Replanning...")
                rospy.sleep(1.5)
                self.planning_error += 1
                return False

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
                self.tf_error += 1
                return False

            (traj_drop, frac_drop) = self.planer.compute_cartesian_path([drop_pose.pose],
                                                                        self.eef_step, self.jump_threshold,
                                                                        True)

            if frac_drop < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_drop * 100, 2)) + "%")

            if not (frac_drop == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                self.planning_error += 1
                return False

            userdata.trajectories[4] = traj_drop
            '''
            # Detach object
            traj_lift_endpoint = traj_lift.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_lift.joint_trajectory.joint_names
            start_state.joint_state.position = traj_lift_endpoint.positions

            rose_primitive = SolidPrimitive()
            rose_primitive.type = 3 #CYLINDER
            rose_height = 0.4
            rose_radius = 0.05
            rose_primitive.dimensions.append(rose_height)
            rose_primitive.dimensions.append(rose_radius)

            rose_pose = Pose()
            rose_pose.orientation.w = 1.0

            rose_collision = CollisionObject()
            rose_collision.header.frame_id = "gripper_"+userdata.active_arm+"_grasp_link"
            rose_collision.id = "current_rose"
            rose_collision.primitives.append(rose_primitive)
            rose_collision.primitive_poses.append(rose_pose)
            rose_collision.operation = 0 #ADD

            rose_attached = AttachedCollisionObject()
            rose_attached.link_name = "gripper_"+userdata.active_arm+"_grasp_link"
            rose_attached.object = rose_collision
            rose_attached.touch_links = ["gripper_"+userdata.active_arm+"_base_link", "gripper_"+userdata.active_arm+"_camera_link", "gripper_"+userdata.active_arm+"_finger_1_link", "gripper_"+userdata.active_arm+"_finger_2_link", "gripper_"+userdata.active_arm+"_grasp_link", "gripper_"+userdata.active_arm+"_palm_link"]

            start_state.attached_collision_objects.append(rose_attached)

            start_state.is_diff = True

            rospy.loginfo("Add an object to world")
            collision_object = CollisionObject()
            collision_object.header.stamp = rospy.Time.now()
            collision_object.header.frame_id = "odom_combined"
            collision_object.id = "object"
            object_shape = SolidPrimitive()
            object_shape.type = 3  # Cylinder
            object_shape.dimensions.append(0.2)  # Height
            object_shape.dimensions.append(0.01)  # Radius
            collision_object.primitives.append(object_shape)
            self.add_remove_object("remove", collision_object, "", "")
            position = [userdata.target[0] + 0.02, userdata.target[1], userdata.target[2], 1.0]
            self.add_remove_object("add", collision_object, position, "primitive")
            '''

            # ----------- RETREAT -----------
            self.traj_name = "retreat"
            traj_drop_endpoint = traj_drop.joint_trajectory.points[-1]
            start_state = RobotState()
            start_state.joint_state.name = traj_drop.joint_trajectory.joint_names
            start_state.joint_state.position = traj_drop_endpoint.positions
            start_state.is_diff = True
            self.planer.set_start_state(start_state)

            retreat_pose_offset = PoseStamped()
            retreat_pose_offset.header.frame_id = "current_object"
            retreat_pose_offset.header.stamp = rospy.Time(0)
            retreat_pose_offset.pose.position.x = -0.12
            retreat_pose_offset.pose.orientation.w = 1
            try:
                retreat_pose = self.tf_listener.transformPose("odom_combined", retreat_pose_offset)
            except Exception, e:
                userdata.error_message = "Could not transform pose. Exception: " + str(e)
                self.tf_error += 1
                return False

            (traj_retreat, frac_retreat) = self.planer.compute_cartesian_path([retreat_pose.pose],
                                                                              self.eef_step, self.jump_threshold,
                                                                              True)

            if frac_retreat < 1.0:
                rospy.logwarn("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")
            else:
                rospy.loginfo("Plan " + self.traj_name + ": " + str(round(frac_retreat * 100, 2)) + "%")

            if not (frac_retreat == 1.0):
                userdata.error_message = "Unable to plan " + self.traj_name + " trajectory"
                self.planning_error += 1
                return False

            userdata.trajectories[5] = traj_retreat
            userdata.target_cs = 0
            rospy.loginfo("Place planning complete")

        # ----------- TRAJECTORY OPERATIONS -----------
        rospy.loginfo("Smooth trajectories")
        try:
            userdata.trajectories[0] = smooth_cartesian_path(userdata.trajectories[0])
            userdata.trajectories[1] = smooth_cartesian_path(userdata.trajectories[1])
            userdata.trajectories[2] = smooth_cartesian_path(userdata.trajectories[2])
            userdata.trajectories[3] = smooth_cartesian_path(userdata.trajectories[3])
            userdata.trajectories[4] = smooth_cartesian_path(userdata.trajectories[4])
            userdata.trajectories[5] = smooth_cartesian_path(userdata.trajectories[5])
        except (ValueError, IndexError, AttributeError):
            userdata.trajectories[:] = []
            userdata.trajectories = range(6)
            userdata.error_message = "Error: " + str(AttributeError)
            self.planning_error += 1
            return False

        rospy.loginfo("Fix velocities")
        userdata.trajectories[0] = fix_velocities(userdata.trajectories[0])
        userdata.trajectories[1] = fix_velocities(userdata.trajectories[1])
        userdata.trajectories[2] = fix_velocities(userdata.trajectories[2])
        userdata.trajectories[3] = fix_velocities(userdata.trajectories[3])
        userdata.trajectories[4] = fix_velocities(userdata.trajectories[4])
        userdata.trajectories[5] = fix_velocities(userdata.trajectories[5])

        # ----------- EXECUTE -----------
        rospy.loginfo("---- Start execution ----")
        rospy.loginfo("Approach")
        self.planer.execute(userdata.trajectories[0])
        # move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("Grasp")
        self.planer.execute(userdata.trajectories[1])
        # move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("Lift")
        self.planer.execute(userdata.trajectories[2])
        rospy.loginfo("Move")
        self.planer.execute(userdata.trajectories[3])
        rospy.loginfo("Drop")
        self.planer.execute(userdata.trajectories[4])
        # move_gripper("gripper_" + userdata.active_arm, "open")
        rospy.loginfo("Retreat")
        self.planer.execute(userdata.trajectories[5])
        # move_gripper("gripper_" + userdata.active_arm, "close")
        rospy.loginfo("---- Execution finished ----")

        # ----------- CLEAR TRAJECTORY LIST -----------
        userdata.trajectories[:] = []
        userdata.trajectories = range(6)

        return True


class SwitchArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'finished', 'switch_targets'],
                             input_keys=['active_arm', 'cs_data', 'target_right', 'target_left'],
                             output_keys=['active_arm', 'cs_data', 'target', 'target_cs'])

        self.counter = 1

    def execute(self, userdata):
        if self.counter == 2:
            return "finished"
        else:
            if self.counter % 2 == 0:
                userdata.target_cs = 0
                userdata.cs_data[2] = 0.0
                self.counter += 1.0
                return "switch_targets"
            if userdata.active_arm == "left":
                userdata.active_arm = "right"
                userdata.cs_data[3] = 1.0
                userdata.target = userdata.target_right
            elif userdata.active_arm == "right":
                userdata.active_arm = "left"
                userdata.cs_data[3] = -1.0
                userdata.target = userdata.target_left
            userdata.target_cs = 0
            userdata.cs_data[2] = 0.0
            self.counter += 1.0
            return "succeeded"


class SwitchTargets(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['target_right', 'target_left', 'active_arm'],
                             output_keys=['target_right', 'target_left', 'target'])

    def execute(self, userdata):
        temp = range(2)
        temp[0] = userdata.target_right[0]
        temp[1] = userdata.target_right[1]
        userdata.target_right[0] = temp[1]
        userdata.target_right[1] = temp[0]
        temp[0] = userdata.target_left[0]
        temp[1] = userdata.target_left[1]
        userdata.target_left[0] = temp[1]
        userdata.target_left[1] = temp[0]
        if userdata.active_arm == "left":
            userdata.target = userdata.target_left
        elif userdata.active_arm == "right":
            userdata.target = userdata.target_right
        return "succeeded"


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

        self.userdata.active_arm = active_arm
        self.userdata.target_cs = 0

        self.userdata.cs_data = range(4)
        self.userdata.cs_data[0] = 0.0  # roll (x)
        self.userdata.cs_data[1] = 0.0  # pitch (y)
        self.userdata.cs_data[2] = -5.0 / 180.0 * math.pi  # yaw (z)
        self.userdata.cs_data[3] = 1.0  # direction for rotation

        self.userdata.target_right = [[]]
        self.userdata.target_right = range(2)  # list for right arm
        self.userdata.target_right[0] = range(3)
        self.userdata.target_right[1] = range(3)

        self.userdata.target_left = [[]]
        self.userdata.target_left = range(2)  # list for left arm
        self.userdata.target_left[0] = range(3)
        self.userdata.target_left[1] = range(3)

        # Pick position for right arm
        self.userdata.target_right[0][0] = 0.6  # x
        self.userdata.target_right[0][1] = -0.3  # y
        self.userdata.target_right[0][2] = 0.7  # z

        # Place position for right arm
        self.userdata.target_right[1][0] = 0.6  # x
        self.userdata.target_right[1][1] = 0.0  # y
        self.userdata.target_right[1][2] = 0.7  # z

        # Pick position for left arm
        self.userdata.target_left[0][0] = 0.6  # x
        self.userdata.target_left[0][1] = 0.0  # y
        self.userdata.target_left[0][2] = 0.7  # z
        
        # Place position for left arm
        self.userdata.target_left[1][0] = 0.6  # x
        self.userdata.target_left[1][1] = 0.3  # y
        self.userdata.target_left[1][2] = 0.7  # z

        self.userdata.trajectories = [0.0]*6  # list for trajectories

        self.userdata.target = self.userdata.target_right

        # Error Counter
        self.userdata.error_tf_max = 60
        self.userdata.error_plan_max = 30
        self.userdata.error_message = ""

        with self:

            smach.StateMachine.add('SPAWN_OBJECTS', SpawnObjects(),
                                   transitions={'succeeded': 'START_POSITION'})

            smach.StateMachine.add('START_POSITION', StartPosition(),
                                   transitions={'succeeded': 'MANIPULATION',
                                                'failed': 'START_POSITION',
                                                'error': 'ERROR'})

            smach.StateMachine.add('MANIPULATION', Manipulation(),
                                   transitions={'succeeded': 'END_POSITION',
                                                'failed': 'MANIPULATION',
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
                                   transitions={'finished': 'ended'})


if __name__ == '__main__':
    rospy.init_node('grasping_test')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()