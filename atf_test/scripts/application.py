#!/usr/bin/python
import unittest
import rospy
import rostest
import tf
import math
import sys

import atf_core
from atf_msgs.msg import MetricResult

class Application:
    def __init__(self):
        self.atf = atf_core.ATF()

        self.pub_freq = 20.0 # Hz
        self.br = tf.TransformBroadcaster()

    def execute(self):

        # small testblock (circle r=0.5, time=3)
        self.atf.start("testblock_small")
        self.pub_tf_circle("link1", "link2", radius=1, time=3)

        # user result
        metric_result = MetricResult()
        metric_result.data = 0.8
        metric_result.groundtruth_result = True
        metric_result.groundtruth_error_message = "all ok in application of atf_test"
        self.atf.stop("testblock_small", metric_result)

        # large testblock (circle r=1, time=5)
        self.atf.start("testblock_large")
        self.pub_tf_circle("link1", "link2", radius=2, time=5)

        # user result
        metric_result = MetricResult()
        metric_result.data = 0.7
        self.atf.stop("testblock_large", metric_result)

        # shutdown atf
        self.atf.shutdown()

    def pub_tf_circle(self, parent_frame_id, child1_frame_id, radius=1, time=1):
        rate = rospy.Rate(int(self.pub_freq))
        for i in range(int(self.pub_freq * time) + 1):
            t = i / self.pub_freq / time
            self.br.sendTransform(
                    (-radius * math.cos(2 * math.pi * t) + radius, -radius * math.sin(2 * math.pi * t), 0),
                    tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time.now(),
                    child1_frame_id,
                    parent_frame_id)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('test_app')
    app = Application()
    app.execute()
