#!/usr/bin/env python
import rospy
import time


class CalculateDistanceToObstacles:
    def __init__(self):
        self.active = False
        self.activation_time = rospy.Time()
        self.distance = 0.0

        rospy.Subscriber("/testing/" )

    def start(self):
        self.active = True
        self.activation_time = rospy.Time(time.time())

    def stop(self):
        self.active = False

    def get_result(self):
        return self.activation_time.to_sec(), ["Distance to obstacles"], [self.distance]
