#!/usr/bin/env python
import sys
import rospy

from atf_testing import ATF, Testblock
from atf_metrics import CalculatePathLength, CalculateTime


if __name__ == '__main__':
    rospy.init_node("atf")

    error = 0
    finished = 0
    
    # Define metrics
    L1 = CalculatePathLength("base_link", "gripper_right_grasp_link")
    T1 = CalculateTime()
    T2 = CalculateTime()
    T3 = CalculateTime()
    T4 = CalculateTime()
    T5= CalculateTime()
    T6 = CalculateTime()
    T7 = CalculateTime()
    T8 = CalculateTime()
    
    test_list = [Testblock("execution_2", [L1, T1]),
                 Testblock("planning_all", [T2]),
                 Testblock("planning_1", [T3]),
                 Testblock("planning_2", [T4]),
                 Testblock("planning_3", [T5]),
                 Testblock("execution_all", [T6]),
                 Testblock("execution_1", [T7]),
                 Testblock("execution_3", [T8])]

    ATF(test_list).wait_for_end()
