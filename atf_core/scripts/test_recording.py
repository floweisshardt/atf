#!/usr/bin/env python
import rospy
import unittest
import rostest

from cob_grasping import SM


class RecordManipulation(unittest.TestCase):

    def test_Manipulation(self):
        sm = SM()
        sm.execute()

if __name__ == '__main__':
    rospy.init_node('record_manipulation')
    rostest.rosrun("atf_core", 'record_manipulation', RecordManipulation, sysargs=None)
