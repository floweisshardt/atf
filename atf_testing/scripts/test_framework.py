#!/usr/bin/env python
import sys
import rospy

from atf_testing import ATF
from atf_metrics import CalculatePathLength, CalculateTime
from atf_msgs.msg import Status

if __name__ == '__main__':
    rospy.init_node("atf")
    
    # Define metrics
    M1 = CalculatePathLength("base_link", "gripper_right_grasp_link")
    T1 = CalculateTime()
    
    # Testblock 'execution_2'
    atf = ATF("execution_2", [M1, T1])

    if atf.get_state() != Status.FINISHED:
        print "An error occured during analysis, no useful results available. State was", atf.get_state()
        sys.exit()
    
    # print results
    print M1.get_path_length()
    print T1.get_time()
