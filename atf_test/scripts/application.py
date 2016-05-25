#!/usr/bin/python
import unittest
import yaml
from subprocess import call

import rospy
import rostest
import rostopic
import tf
import math
from atf_recorder import RecordingManager

class Application:
    def __init__(self):
        # ATF code
        self.testblock_small = RecordingManager('testblock_small')
        self.testblock_large = RecordingManager('testblock_large')
        self.testblock_all = RecordingManager('testblock_all')
        
        # native app code
        self.pub_freq = 100.0 # Hz
        self.br = tf.TransformBroadcaster()
        rospy.sleep(1) #wait for tf broadcaster to get active (rospy bug?)

    def execute(self):
        self.testblock_all.start()

        # small testblock (circle r=0.5, time=3)
        self.testblock_small.start()
        self.pub_tf_circle("link1", "link2", radius=1, time=3)
        self.testblock_small.stop()

        # large testblock (circle r=1, time=5
        self.testblock_large.start()
        self.pub_tf_circle("link1", "link2", radius=2, time=5)
        self.testblock_large.stop()

        self.testblock_all.stop()

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

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('test_name')
    rostest.rosrun('application', 'recording', Test, sysargs=None)
