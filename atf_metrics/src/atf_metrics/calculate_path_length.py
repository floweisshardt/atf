#!/usr/bin/env python
import math
import rospy
import tf


class CalculatePathLength:
    def __init__(self, root_frame, measured_frame):
        
        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.path_length = 0
        self.tf_sampling_freq = 100.0  # Hz
        self.first_value = True
        self.trans_old = []
        self.rot_old = []
        
        self.listener = tf.TransformListener()

        # call tf recording cyclically
        rospy.Timer(rospy.Duration.from_sec(1/self.tf_sampling_freq), self.record_tf)

    def start(self):
        self.active = True
        rospy.loginfo("Start measurement for transformation " + self.root_frame + " ---> " + self.measured_frame)

    def stop(self):
        self.active = False
        rospy.loginfo("Stop measurement for transformation " + self.root_frame + " ---> " + self.measured_frame)

    def record_tf(self, event):
        if self.active:
            try:

                self.listener.waitForTransform(self.root_frame,
                                               self.measured_frame,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(2/self.tf_sampling_freq))
                (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            except (tf.Exception, tf.LookupException, tf.ConnectivityException), e:
                rospy.logerr(e)
            else:
                if self.first_value:
                    self.trans_old = trans
                    self.rot_old = rot
                    self.first_value = False
                    return

                path_increment = math.sqrt((trans[0] - self.trans_old[0])**2 + (trans[1] - self.trans_old[1])**2 +
                                           (trans[2] - self.trans_old[2])**2)
                self.path_length += path_increment

                self.trans_old = trans
                self.rot_old = rot

    def get_path_length(self):
        return self.path_length
