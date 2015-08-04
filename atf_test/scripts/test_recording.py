#!/usr/bin/env python
import rospy
import unittest
import rostest

from atf_test import PublishTf


class TestRecording(unittest.TestCase):

    def test_Recording(self):
        PublishTf()

if __name__ == '__main__':
    rospy.init_node('test_template')
    rostest.rosrun("atf_test", 'test1_recording', TestRecording, sysargs=None)
