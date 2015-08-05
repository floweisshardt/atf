#!/usr/bin/env python
import rospy
import unittest
import rostest

from cob_grasping import SM


class TestRecording(unittest.TestCase):

    def setUp(self):
        self.sm = SM()

    def test_Recording(self):

        self.sm.execute()

if __name__ == '__main__':
    rospy.init_node('test_recording')
    rostest.rosrun("atf_core", 'test_recording', TestRecording, sysargs=None)
