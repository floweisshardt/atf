#!/usr/bin/env python
import rospy
import time


class CalculateDistanceToObstacles:
    def __init__(self, testblock):
        self.active = False
        self.finished = False
        self.testblock = testblock
        self.activation_time = rospy.Time()
        self.distance = 0.0

        rospy.Subscriber("/testing/" + self.testblock + "/CollisionDistance", float, self.get_distance)

    def start(self):
        self.active = True
        self.activation_time = rospy.Time(time.time())

    def stop(self):
        self.active = False
        self.finished = True

    def get_distance(self, data):
        if self.active:
            self.distance = data

    def get_result(self):
        if self.finished:
            return self.activation_time.to_sec(), "Distance to obstacles", self.distance
        else:
            return False
