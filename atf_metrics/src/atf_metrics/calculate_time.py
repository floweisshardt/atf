#!/usr/bin/env python
import rospy


class CalculateTime:
    def __init__(self):
        self.active = False

    def start(self):
        self.active = True
        rospy.loginfo("Start measurement for time")

    def stop(self):
        self.active = False
        rospy.loginfo("Stop measurement for time")