#!/usr/bin/env python

import math

import rospy
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from atf_msgs.msg import Status, Trigger

class TestingTf():
    def __init__(self, root_frame, measured_frame):
        
        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        
        self.listener = tf.TransformListener()
        
        self.tf_sampling_time = 0.1 # sec
        self.listener.waitForTransform(self.measured_frame, self.root_frame, rospy.Time(0), rospy.Duration(3.0))
        rospy.Timer(rospy.Duration(self.tf_sampling_time), self.record_tf)
        self.current_pose = PoseStamped()
        self.path_length = 0

            
    def start(self):
        self.active = True
        print "start ttf for transfromation", self.root_frame, "--->", self.measured_frame

    def stop(self):
        self.active = False
        print "stop ttf for transformation", self.root_frame, "--->", self.measured_frame

    def record_tf(self, event):
        try:
            now = rospy.Time.now() - rospy.Duration(self.tf_sampling_time)
            #self.listener.waitForTransform("base_laser_front_link", "reference", now, rospy.Duration(2.0 * self.tf_sampling_time))
            (trans_old, rot_old) = self.listener.lookupTransform(self.root_frame, self.measured_frame, now)
            (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            path_increment = math.sqrt((trans[0] - trans_old[0])**2 + (trans[1] - trans_old[1])**2 + (trans[2] - trans_old[2])**2)
            
            if self.active:
                self.path_length += path_increment

        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            print e

    def tf_distance(self):
        print "tf distance"
        
        pose_desired = PoseStamped()
        pose_desired.header.frame_id = "base_laser_front_link"
        pose_desired.pose.position.x = 1.1
        pose_desired.pose.position.y = 0.5
        pose_desired.pose.position.z = 0



        try:
            pose_diff = self.listener.transformPose("reference", pose_desired)
            #print pose_diff
        
            #(trans,rot) = self.listener.lookupTransform('base_laser_front_link', 'reference', rospy.Time(0))
            #print trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e
            return False
        
        trans_diff = math.sqrt(pose_diff.pose.position.x**2 + pose_diff.pose.position.y**2 + pose_diff.pose.position.z**2)
        print "trans_diff =", trans_diff

    def get_path_length(self):
        return self.path_length


if __name__ == '__main__':
    TTf = TestingTf()
    rospy.spin()
    #TTf.tf_distance()
    #TTf.tf_path()
