#!/usr/bin/env python

import sys
import math

import rospy
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from atf_msgs.msg import Status, Trigger

from atf import ATF
from testing_tf import TestingTf

if __name__ == '__main__':
    rospy.init_node("atf")
    
    ### define metrics
    # define one test for tf path length
    # TTf1 = TestingTf("base_laser_front_link", "gripper_right_grasp_link")
    TTf1 = TestingTf("base_laser_front_link", "reference1")
    TTf2 = TestingTf("base_laser_front_link", "reference2")
    TTf3 = TestingTf("base_laser_front_link", "reference3")
    TTf4 = TestingTf("base_laser_front_link", "reference4")
    
    # initialize atf with metrics
    atf = ATF([TTf1, TTf2, TTf3, TTf4])
    # atf = ATF([TTf1])
        
    if atf.get_state() != Status.FINISHED:
        print "an error occured during analysis, no useful results available. state was", atf.get_state()
        sys.exit()
    
    # print results
    print TTf1.get_path_length()
    print TTf2.get_path_length()
    print TTf3.get_path_length()
    print TTf4.get_path_length()
