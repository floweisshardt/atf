#!/usr/bin/env python
import rospy


class CalculateDistanceToObstaclesParamHandler:
    def __init__(self):
        self.params = ""

    @staticmethod
    def parse_parameter(params):

        return CalculateDistanceToObstacles(params)


class CalculateDistanceToObstacles:
    def __init__(self, testblock):
        self.active = False
        self.finished = False
        self.testblock = testblock
        self.distance = 0.0

        rospy.Subscriber("/testing/" + self.testblock + "/CollisionDistance", float, self.get_distance)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    def get_distance(self, data):
        if self.active:
            self.distance = data

    def get_result(self):
        if self.finished:
            return "Distance to obstacles", self.distance
        else:
            return False
