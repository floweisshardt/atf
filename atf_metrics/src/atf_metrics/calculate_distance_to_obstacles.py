#!/usr/bin/env python
import rospy
from atf_msgs.msg import ObstacleDistance
from types import *


class CalculateDistanceToObstaclesParamHandler:
    def __init__(self):
        self.params = ""

    @staticmethod
    def parse_parameter(params):

        metrics = []

        for item in params:
            try:
                metrics.append(CalculateDistanceToObstacles(item[0], item[1]))
            except IndexError:
                metrics.append(CalculateDistanceToObstacles(item[0]))

        return metrics


class CalculateDistanceToObstacles:
    def __init__(self, links, collision_object="all"):
        self.active = False
        self.finished = False
        self.links = links
        self.collision_objects = collision_object
        self.distances = {}
        self.start_chain = False

        rospy.Subscriber("/atf/obstacle_distance", ObstacleDistance, self.get_distance)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    def pause(self):
        self.active = False

    @staticmethod
    def purge():
        pass

    def get_distance(self, data):
        if self.active:
            for link in data.links:
                # Single link
                if type(self.links) is not ListType:
                    if link.name == self.links:
                        for idx, co in enumerate(link.objects):
                            if not self.collision_objects == "all":
                                if co in self.collision_objects:
                                    self.save_result(self.distances, link.name, co, link.distances[idx])
                            else:
                                self.save_result(self.distances, link.name, co, link.distances[idx])
                        break
                else:
                    # Link chain
                    if link.name == self.links[0]:
                        self.start_chain = True
                    if self.start_chain:
                        for idx, co in enumerate(link.objects):
                            if not self.collision_objects == "all":
                                if co in self.collision_objects:
                                    self.save_result(self.distances, link.name, co, link.distances[idx])
                            else:
                                self.save_result(self.distances, link.name, co, link.distances[idx])
                    if link.name == self.links[1]:
                        self.start_chain = False
                        break

    @staticmethod
    def save_result(save, link_name, co, distance):
        if link_name + " to " + co not in save:
            save[link_name + " to " + co] = []
        save[link_name + " to " + co].append(distance)

    def get_result(self):
        if self.finished:
            obstacle_distance_minimum = {}

            for distance in self.distances:
                obstacle_distance_minimum[distance] = round(min(self.distances[distance]), 3)

            return "obstacle_distance", obstacle_distance_minimum
        else:
            return False
