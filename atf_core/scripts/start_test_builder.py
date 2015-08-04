#!/usr/bin/env python
import rospy

from atf_core import TestBuilder

if __name__ == '__main__':
    rospy.init_node("test_framework")
    TestBuilder()
