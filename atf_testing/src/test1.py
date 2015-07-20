#!/usr/bin/env python

import sys
import rospy

from atf_msgs.msg import Status

from atf import ATF
from atf_metrics.calculate_path_length import CalculatePathLength

if __name__ == '__main__':
    rospy.init_node("atf")
    
    # define metrics
    # define one test for tf path length
    TTf1 = CalculatePathLength("base_link", "gripper_right_grasp_link")
    # TTf1 = CalculatePathLength("base_laser_front_link", "reference1")
    # TTf2 = CalculatePathLength("base_laser_front_link", "reference2")
    # TTf3 = CalculatePathLength("base_laser_front_link", "reference3")
    # TTf4 = CalculatePathLength("base_laser_front_link", "reference4")
    
    # initialize atf with metrics
    # atf = ATF([TTf1, TTf2, TTf3, TTf4])
    atf = ATF([TTf1])

    if atf.get_state() != Status.FINISHED:
        print "An error occured during analysis, no useful results available. State was", atf.get_state()
        sys.exit()
    
    # print results
    print TTf1.get_path_length()
    # print TTf2.get_path_length()
    # print TTf3.get_path_length()
    # print TTf4.get_path_length()
