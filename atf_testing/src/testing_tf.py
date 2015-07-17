#!/usr/bin/env python

import math

import rospy
import tf

from geometry_msgs.msg import PoseStamped


class TestingTf:
    def __init__(self, root_frame, measured_frame):
        
        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.path_length = 0
        self.current_pose = PoseStamped()
        self.tf_sampling_time = 0.01  # sec
        
        self.listener = tf.TransformListener()

        # wait for tf listener to be connected
        try:
            self.listener.waitForTransform(self.measured_frame, self.root_frame, rospy.Time(0),
                                           rospy.Duration.from_sec(3.0))
        except tf.Exception, e:
            rospy.logerr(e)

        (self.trans_old, self.rot_old) = self.listener.lookupTransform(self.root_frame, self.measured_frame,
                                                                       rospy.Time(0))
        # call tf recording cyclically
        rospy.Timer(rospy.Duration.from_sec(self.tf_sampling_time), self.record_tf)

    def start(self):
        self.active = True
        print "start ttf for transformation", self.root_frame, "--->", self.measured_frame

    def stop(self):
        self.active = False
        print "stop ttf for transformation", self.root_frame, "--->", self.measured_frame

    def record_tf(self, event):
        try:

            self.listener.waitForTransform(self.root_frame,
                                           self.measured_frame,
                                           rospy.Time(0),
                                           rospy.Duration.from_sec(2.0 * self.tf_sampling_time))
            (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            path_increment = math.sqrt((trans[0] - self.trans_old[0])**2 +
                                       (trans[1] - self.trans_old[1])**2 +
                                       (trans[2] - self.trans_old[2])**2)
            if self.active:
                self.path_length += path_increment

            self.trans_old = trans
            self.rot_old = rot

        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            print e

    def get_path_length(self):
        return self.path_length

if __name__ == '__main__':
    TTf = TestingTf()
    rospy.spin()
