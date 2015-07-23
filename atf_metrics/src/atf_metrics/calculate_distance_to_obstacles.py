#!/usr/bin/env python
import rospy


class CalculateDistanceToObstacles:
    def __init__(self):
        self.active = False

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def get_result(self):
        return ""
