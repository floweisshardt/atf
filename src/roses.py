#!/usr/bin/python
import rospy
import rospkg
import smach
import smach_ros
import tf
import sys
import copy

import random

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from simple_script_server import *
sss = simple_script_server()

class RotateRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['active_arm'])
        self.angle_offset_yaw = 0
        self.angle_offset_roll = 0
        self.rose_position_y = 0.3
        self.direction_yaw = 1
        
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        
    def broadcast_tf(self, event):
        self.br.sendTransform(
            (0.6, self.rose_position_y, 0.8),
            tf.transformations.quaternion_from_euler(self.angle_offset_roll, 0, self.angle_offset_yaw),
            event.current_real,
            "current_rose",
            "base_link")

    def execute(self, userdata):
        if self.angle_offset_yaw >= 0.5 * math.pi:
            self.direction_yaw = -1;
        elif self.angle_offset_yaw <= -0.5 * math.pi:
            self.direction_yaw = 1;

        if self.direction_yaw == 1:
            self.angle_offset_yaw += 5.0 / 180.0 * math.pi;
        elif self.direction_yaw == -1:
            self.angle_offset_yaw -= 5.0 / 180.0 * math.pi;

        if userdata.active_arm == "left":
            self.angle_offset_roll = math.pi
            self.rose_position_y = 0.3;
        elif userdata.active_arm == "right":
            self.angle_offset_roll = 0
            self.rose_position_y = -0.3;
        return "succeeded"

class GraspRose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','failed'],
            input_keys=['active_arm'])
        
        # initialize tf listener
        self.listener = tf.TransformListener()
        
        ### Create a handle for the Move Group Commander
        self.mgc_left = MoveGroupCommander("arm_left")
        self.mgc_right = MoveGroupCommander("arm_right")
        
        ### Create a handle for the Planning Scene Interface
        self.psi = PlanningSceneInterface()
        
        self.eef_step = 0.01
        self.jump_threshold = 2
        
        rospy.sleep(1)
        
    def execute(self, userdata):

        if not self.plan_and_execute(userdata):
            return "failed"
        
        return "succeeded"

    def plan_and_execute(self, userdata):

        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm,"pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return False
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_rose"
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.12
        approach_pose_offset.pose.orientation.w = 1
        try:
            approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False
        
        if userdata.active_arm == "left":
            (traj_approach,frac_approach) = self.mgc_left.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_approach,frac_approach) = self.mgc_right.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_approach = self.smooth_cartesian_path(traj_approach)
        
        print frac_approach
        
        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory")
            #sss.say(["no approach trajectory: skipping rose"], False)
            return False

        ### Set next (virtual) start state
        traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
        start_state.joint_state.position = traj_approach_endpoint.positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_rose"
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        if userdata.active_arm == "left":
            (traj_grasp,frac_grasp) = self.mgc_left.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_grasp,frac_grasp) = self.mgc_right.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_grasp = self.smooth_cartesian_path(traj_grasp)
        
        print frac_grasp
        
        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory")
            #sss.say(["no grasp trajectory: skipping rose"], False)
            return False

        ### Set next (virtual) start state
        traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
        start_state.joint_state.position = traj_grasp_endpoint.positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_rose"
        lift_pose_offset.header.stamp = rospy.Time(0)
        if userdata.active_arm == "left":
            lift_pose_offset.pose.position.z = -0.2#-0.2#-0.3#-0.12
        elif userdata.active_arm == "right":
            lift_pose_offset.pose.position.z = 0.2#0.3#0.12
        else:
            rospy.logerr("invalid active_arm: %s", userdata.active_arm)
            sys.exit()
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        if userdata.active_arm == "left":
            (traj_lift,frac_lift) = self.mgc_left.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_lift,frac_lift) = self.mgc_right.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False
        
        traj_lift = self.smooth_cartesian_path(traj_lift)
        
        print frac_lift
        
        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            #sss.say(["no lift trajectory: skipping rose"], False)
            return False

        #if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0 and not traj_pre_grasp == None):
        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            sss.say(["skipping rose"], False)
            return False
        else:
            sss.say(["grasping rose"], False)

            # fix trajectories to stop at the end
            traj_approach.joint_trajectory.points[-1].velocities = [0]*7
            traj_grasp.joint_trajectory.points[-1].velocities = [0]*7
            traj_lift.joint_trajectory.points[-1].velocities = [0]*7
            
            # fix trajectories to be slower
            speed_factor = 1
            for i in range(len(traj_approach.joint_trajectory.points)):
                traj_approach.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_grasp.joint_trajectory.points)):
                traj_grasp.joint_trajectory.points[i].time_from_start *= speed_factor
            for i in range(len(traj_lift.joint_trajectory.points)):
                traj_lift.joint_trajectory.points[i].time_from_start *= speed_factor

            ### execute
            #sss.wait_for_input()
            sss.move("arm_" + userdata.active_arm, "pre_grasp")
            #sss.wait_for_input()
            rospy.loginfo("approach")
            if userdata.active_arm == "left":
                self.mgc_left.execute(traj_approach)
                #handle_gripper = sss.move("gripper_" + userdata.active_arm, "open")
                #move_gripper("gripper_" + userdata.active_arm, "open")
                #sss.wait_for_input()
                rospy.loginfo("grasp")
                self.mgc_left.execute(traj_grasp)
                #sss.wait_for_input()
                #sss.move("gripper_" + userdata.active_arm, "close")
                #move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_left.execute(traj_lift)
                #sss.wait_for_input()
                #self.mgc_left.execute(traj_pre_grasp)
                #rospy.sleep(1)
                #sss.move("base","middle", mode="linear", blocking=False)
                #rospy.sleep(0.5) #wait for base to move away from table
                handle_arm = sss.move("arm_" + userdata.active_arm, "retreat")
            elif userdata.active_arm == "right":
                self.mgc_right.execute(traj_approach)
                #sss.move("gripper_" + userdata.active_arm, "open")
                #move_gripper("gripper_" + userdata.active_arm, "open")
                #sss.wait_for_input()
                rospy.loginfo("grasp")
                self.mgc_right.execute(traj_grasp)
                #sss.wait_for_input()
                #sss.move("gripper_" + userdata.active_arm, "close")
                #move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_right.execute(traj_lift)
                #sss.wait_for_input()
                #self.mgc_right.execute(traj_pre_grasp)
                #rospy.sleep(1)
                #sss.move("base","middle", mode="linear", blocking=False)
                #rospy.sleep(0.5) #wait for base to move away from table
                handle_arm = sss.move("arm_" + userdata.active_arm, "retreat")
            else:
                rospy.logerr("invalid arm_active")
                return False

        return True
        
    def smooth_cartesian_path(self, traj):
        #print traj
        time_offset = 0.2
        
        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start += rospy.Duration(time_offset)
        
        traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(time_offset)
        
        
        #print "\n\n\n"
        #print traj
        return traj



class SM(smach.StateMachine):
    def __init__(self):        
        smach.StateMachine.__init__(self,outcomes=['ended'])
        
        self.userdata.active_arm = 'right'
        
        with self:
            smach.StateMachine.add('GRASP',GraspRose(),
                transitions={'succeeded':'ended',
                    'failed':'ROTATE_ROSE'})
            smach.StateMachine.add('ROTATE_ROSE',RotateRose(),
                transitions={'succeeded':'GRASP'})

if __name__=='__main__':
    rospy.init_node('roses')    
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
