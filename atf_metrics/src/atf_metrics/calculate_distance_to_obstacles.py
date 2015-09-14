#!/usr/bin/env python
import rospy
from atf_msgs.msg import ObstacleDistance


class CalculateDistanceToObstaclesParamHandler:
    def __init__(self):
        self.params = ""

    @staticmethod
    def parse_parameter(params):

        metrics = []

        for item in params:
            metrics.append(CalculateDistanceToObstacles(item[0], item[1]))

        return metrics


class CalculateDistanceToObstacles:
    def __init__(self, links, collision_object):
        self.active = False
        self.finished = False
        self.links = links
        self.collision_objects = collision_object
        self.distances = {}

        rospy.Subscriber("/atf/obstacle_distance", ObstacleDistance, self.get_distance)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    def get_distance(self, data):
        if self.active:
            for link in data.links:
                if link.name == self.links:
                    for idx, co in enumerate(link.objects):
                        if co in self.collision_objects:
                            if link.name + " to " + co not in self.distances:
                                self.distances[link.name + " to " + co] = []

                            self.distances[link.name + " to " + co].append(link.distances[idx])

    def get_result(self):
        if self.finished:
            obstacle_distance_minimum = {}

            for distance in self.distances:
                obstacle_distance_minimum[distance] = round(min(self.distances[distance]), 3)

            return "obstacle_distance", obstacle_distance_minimum
        else:
            return False
