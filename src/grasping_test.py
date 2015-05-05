#!/usr/bin/python
import rospy
import rospkg
import smach
import smach_ros
import tf

from pyassimp import pyassimp

from moveit_commander import MoveGroupCommander
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.msg import RobotTrajectory
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive

from simple_script_server import *
sss = simple_script_server()
active_arm = 'right'


def move_gripper(component_name, pos):
    error_code = -1
    counter = 0
    while not rospy.is_shutdown() and error_code != 0:
        print "trying to move", component_name, "to", pos, "retries: ", counter
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


class RotateCS(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['active_arm'])
        self.angle_offset_yaw = 0

        if active_arm == "left":
            self.angle_offset_roll = math.pi
            self.rose_position_y = 0.3
            self.direction_yaw = -1
        elif active_arm == "right":
            self.angle_offset_roll = 0
            self.rose_position_y = -0.3
            self.direction_yaw = 1
        
        rospy.Timer(rospy.Duration.from_sec(3.0), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        
    def broadcast_tf(self, event):
        self.br.sendTransform(
            (0.6, self.rose_position_y, 0.7),
            quaternion_from_euler(self.angle_offset_roll, 0, self.angle_offset_yaw),
            event.current_real,
            "current_object",
            "base_link")

    def execute(self, userdata):
        if self.angle_offset_yaw >= 0.5 * math.pi:
            self.direction_yaw = -1
        elif self.angle_offset_yaw <= -0.5 * math.pi:
            self.direction_yaw = 1

        if self.direction_yaw == 1:
            self.angle_offset_yaw += 5.0 / 180.0 * math.pi
        elif self.direction_yaw == -1:
            self.angle_offset_yaw -= 5.0 / 180.0 * math.pi

        if userdata.active_arm == "left":
            self.angle_offset_roll = math.pi
            self.rose_position_y = 0.3
        elif userdata.active_arm == "right":
            self.angle_offset_roll = 0
            self.rose_position_y = -0.3
        return "succeeded"


class Grasping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed'],
                             input_keys=['active_arm', 'pre_grasp', 'retreat'],
                             output_keys=['pre_grasp', 'retreat'])

        # initialize tf listener
        self.listener = tf.TransformListener()

        # Create a handle for the Move Group Commander
        self.mgc_left = MoveGroupCommander("arm_left")
        self.mgc_right = MoveGroupCommander("arm_right")

        self.traj_pre_grasp = RobotTrajectory()
        self.traj_retreat = RobotTrajectory()

        # Create a scene publisher to push changes to the scene
        self.pub_planning_scene = rospy.Publisher("/planning_scene", PlanningScene, queue_size=1)

        self.planning_scene = PlanningScene()
        self.planning_scene.is_diff = True

        self.eef_step = 0.01
        self.jump_threshold = 2

        rospy.loginfo("Add table to world")
        self.add_table_to_world()
        rospy.sleep(1)

        rospy.loginfo("Add object to world")
        self.add_object_to_world()
        rospy.sleep(1)

        # move_gripper("gripper_left", "open")
        # move_gripper("gripper_right", "open")

    def add_table_to_world(self):
        remove_object = CollisionObject()
        remove_object.id = "table"
        remove_object.header.stamp = rospy.Time.now()
        remove_object.header.frame_id = "/world"
        remove_object.operation = CollisionObject.REMOVE

        self.planning_scene.world.collision_objects[:] = []
        self.planning_scene.world.collision_objects.append(remove_object)
        self.pub_planning_scene.publish(self.planning_scene)

        collision_object = CollisionObject()

        collision_object.header.stamp = rospy.Time.now()
        collision_object.header.frame_id = "/world"
        collision_object.id = "table"

        filename = rospkg.RosPack().get_path("cob_grasping") + "/files/table.stl"
        collision_object.meshes.append(self.load_mesh(filename))

        mesh_pose = Pose()
        mesh_pose.position.x = 0.37
        mesh_pose.position.y = -0.3
        mesh_pose.position.z = 0.62
        mesh_pose.orientation.w = 1.0
        collision_object.mesh_poses.append(mesh_pose)

        self.planning_scene.world.collision_objects.append(collision_object)
        self.pub_planning_scene.publish(self.planning_scene)

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

    def add_object_to_world(self):
        remove_object = CollisionObject()
        remove_object.id = "object"
        remove_object.header.stamp = rospy.Time.now()
        remove_object.header.frame_id = "/world"
        remove_object.operation = CollisionObject.REMOVE

        self.planning_scene.world.collision_objects[:] = []
        self.planning_scene.world.collision_objects.append(remove_object)
        self.pub_planning_scene.publish(self.planning_scene)

        collision_object = CollisionObject()

        collision_object.header.stamp = rospy.Time.now()
        collision_object.header.frame_id = "/world"
        collision_object.id = "object"
        collision_object.operation = CollisionObject.ADD

        object_shape = SolidPrimitive()
        object_shape.type = 3  # CYLINDER
        object_shape.dimensions.append(0.2)
        object_shape.dimensions.append(0.01)

        object_pose = Pose()
        object_pose.position.x = 0.62
        object_pose.position.y = -0.3
        object_pose.position.z = 0.7
        object_pose.orientation.w = 1.0

        collision_object.primitives.append(object_shape)
        collision_object.primitive_poses.append(object_pose)

        self.planning_scene.world.collision_objects.append(collision_object)
        self.pub_planning_scene.publish(self.planning_scene)

        '''
        rose_collision = CollisionObject()
        rose_collision.header.frame_id = "gripper_"+userdata.active_arm+"_grasp_link"
        rose_collision.id = "current_object"
        rose_collision.primitives.append(rose_primitive)
        rose_collision.primitive_poses.append(rose_pose)
        rose_collision.operation = 0 #ADD
        rose_attached = AttachedCollisionObject()
        rose_attached.link_name = "gripper_"+userdata.active_arm+"_grasp_link"
        rose_attached.object = rose_collision
        rose_attached.touch_links = ["gripper_"+userdata.active_arm+"_base_link","gripper_"+userdata.active_arm+"_camera_link", "gripper_"+userdata.active_arm+"_finger_1_link", "gripper_"+userdata.active_arm+"_finger_2_link", "gripper_"+userdata.active_arm+"_grasp_link", "gripper_"+userdata.active_arm+"_palm_link"]
        start_state.attached_collision_objects.append(rose_attached)

        mesh_pose = Pose()
        mesh_pose.position.x = 0.37
        mesh_pose.position.y = -0.3
        mesh_pose.position.z = 0.62
        mesh_pose.orientation.w = 1.0
        collision_object.mesh_poses.append(mesh_pose)
        '''

    def execute(self, userdata):
        if userdata.active_arm == "left":
            self.planer = self.mgc_left
        elif userdata.active_arm == "right":
            self.planer = self.mgc_right
        else:
            rospy.logerr("invalid arm_active")

        if not self.plan_and_execute(userdata):
            return "failed"

        return "succeeded"

    def plan_movement(self, arm, pose):
        (config, error_code) = sss.compose_trajectory("arm_" + arm, pose)
        if error_code != 0:
            rospy.logerr("unable to parse " + pose + " configuration")

        start_state = RobotState()
        start_state.joint_state.name = config.joint_names

        start_state.joint_state.position = self.planer.get_current_joint_values()
        self.planer.set_start_state(start_state)

        self.planer.clear_pose_targets()
        self.planer.set_joint_value_target(config.points[0].positions)

        plan = self.planer.plan()

        plan = self.smooth_cartesian_path(plan)
        plan = self.scale_joint_trajectory_speed(plan, 0.3)
        return plan

    def plan_and_execute(self, userdata):

        while not userdata.pre_grasp:
            try:
                self.traj_pre_grasp = self.plan_movement(userdata.active_arm, "pre_grasp")
            except (ValueError, IndexError):
                userdata.pre_grasp = False
            else:
                userdata.pre_grasp = True
                rospy.loginfo("pre_grasp")
                self.planer.execute(self.traj_pre_grasp)

        # Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm, "pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return "failed"

        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
        self.planer.set_start_state(start_state)

        # Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_object"
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.12
        approach_pose_offset.pose.orientation.w = 1
        try:
            approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False

        (traj_approach, frac_approach) = self.planer.compute_cartesian_path([approach_pose.pose],
                                                                            self.eef_step, self.jump_threshold, True)

        print "Plan approach: " + str(frac_approach * 100.0) + "%"

        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory")
            return False

        # Set next (virtual) start state
        traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
        start_state.joint_state.position = traj_approach_endpoint.positions
        start_state.is_diff = True
        self.planer.set_start_state(start_state)

        # Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_object"
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        (traj_grasp, frac_grasp) = self.planer.compute_cartesian_path([grasp_pose.pose],
                                                                      self.eef_step, self.jump_threshold, True)

        print "Plan grasp: " + str(frac_grasp * 100.0) + "%"

        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory")
            return False

        # Set next (virtual) start state
        traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
        start_state.joint_state.position = traj_grasp_endpoint.positions
        start_state.is_diff = True
        self.planer.set_start_state(start_state)

        # Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_object"
        lift_pose_offset.header.stamp = rospy.Time(0)
        if userdata.active_arm == "left":
            lift_pose_offset.pose.position.z = -0.2
        elif userdata.active_arm == "right":
            lift_pose_offset.pose.position.z = 0.2
        else:
            rospy.logerr("invalid active_arm: %s", userdata.active_arm)
            sys.exit()
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        (traj_lift, frac_lift) = self.planer.compute_cartesian_path([lift_pose.pose],
                                                                    self.eef_step, self.jump_threshold, True)

        print "Plan lift: " + str(frac_lift * 100.0) + "%"

        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            return False

        else:
            traj_approach = self.smooth_cartesian_path(traj_approach)
            traj_grasp = self.smooth_cartesian_path(traj_grasp)
            traj_lift = self.smooth_cartesian_path(traj_lift)

            traj_approach = self.fix_velocities(traj_approach)
            traj_grasp = self.fix_velocities(traj_grasp)
            traj_lift = self.fix_velocities(traj_lift)

            # execute
            rospy.loginfo("approach")
            self.planer.execute(traj_approach)
            move_gripper("gripper_" + userdata.active_arm, "open")
            rospy.loginfo("grasp")
            self.planer.execute(traj_grasp)
            move_gripper("gripper_" + userdata.active_arm, "close")
            rospy.loginfo("lift")
            self.planer.execute(traj_lift)

            # Plan Retreat
            while not userdata.retreat:
                try:
                    self.traj_retreat = self.plan_movement(userdata.active_arm, "retreat")
                except (ValueError, IndexError):
                    userdata.retreat = False
                else:
                    userdata.retreat = True
                    rospy.loginfo("retreat")
                    self.planer.execute(self.traj_retreat)

        return True

    @staticmethod
    def smooth_cartesian_path(traj):

        # time_offset = 0.2
        time_offset = 12.0

        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start += rospy.Duration.from_sec(time_offset)

        traj.joint_trajectory.points[-1].time_from_start += rospy.Duration.from_sec(time_offset)

        return traj

    @staticmethod
    def fix_velocities(traj):
        # fix trajectories to stop at the end
        traj.joint_trajectory.points[-1].velocities = [0]*7

        # fix trajectories to be slower
        speed_factor = 1.0
        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start *= speed_factor

        return traj

    @staticmethod
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


class SwitchArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'finished'],
                             input_keys=['active_arm'],
                             output_keys=['active_arm', 'pre_grasp', 'retreat'])

        self.counter = 1

    def execute(self, userdata):
        if self.counter == 2:
            return "finished"
        else:
            if userdata.active_arm == "left":
                userdata.active_arm = "right"
            elif userdata.active_arm == "right":
                userdata.active_arm = "left"
            self.counter += 1.0
            userdata.pre_grasp = False
            userdata.retreat = False
            return "succeeded"


class SM(smach.StateMachine):
    def __init__(self):        
        smach.StateMachine.__init__(self, outcomes=['ended'])

        self.userdata.active_arm = active_arm
        self.userdata.pre_grasp = False
        self.userdata.retreat = False
        
        with self:
            smach.StateMachine.add('GRASP', Grasping(),
                                   transitions={'succeeded': 'SWITCH_ARM',
                                                'failed': 'ROTATE_COORDINATESYSTEM'})

            smach.StateMachine.add('ROTATE_COORDINATESYSTEM', RotateCS(),
                                   transitions={'succeeded': 'GRASP'})

            smach.StateMachine.add('SWITCH_ARM', SwitchArm(),
                                   transitions={'succeeded': 'ROTATE_COORDINATESYSTEM',
                                                'finished': 'ended'})

if __name__ == '__main__':
    rospy.init_node('roses')    
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()