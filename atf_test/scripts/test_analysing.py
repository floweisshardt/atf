#!/usr/bin/env python
import sys
import rospy
import unittest
import rostest

from atf_core import TestBuilder


class TestAnalysing(unittest.TestCase):

    def test_Analysing(self):
        TestBuilder()

if __name__ == '__main__':
    rospy.init_node('test_template')
    rostest.rosrun("atf_test", 'test1_analysing', TestAnalysing, sysargs=None)
