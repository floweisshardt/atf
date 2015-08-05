#!/usr/bin/python
import rospy

from cob_grasping import SM

if __name__ == '__main__':
    rospy.init_node('grasping_app')
    sm = SM()
    outcome = sm.execute()
