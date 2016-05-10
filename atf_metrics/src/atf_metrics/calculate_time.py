#!/usr/bin/env python
import rospy
import time
import math


class CalculateTimeParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    def parse_parameter(self, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        try:
            groundtruth = params["groundtruth"]
            groundtruth_epsilon = params["groundtruth_epsilon"]
        except Exception as e:
            print e
            rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation")
            groundtruth = None
            groundtruth_epsilon = None
        return CalculateTime(groundtruth, groundtruth_epsilon)


class CalculateTime:
    def __init__(self, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the time between the trigger 'ACTIVATE' and 'FINISH' on the topic assigned to the
        testblock.
        """
        self.start_time = rospy.Time()
        self.stop_time = rospy.Time()
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False

    def start(self):
        self.start_time = rospy.Time.from_sec(time.time())

    def stop(self):
        self.stop_time = rospy.Time.from_sec(time.time())
        self.finished = True

    @staticmethod
    def pause():
        # TODO: Implement pause time calculation
        pass

    @staticmethod
    def purge():
        pass

    def get_result(self):
        groundtruth_result = False
        if self.finished:
            duration = round((self.stop_time.to_sec() - self.start_time.to_sec()), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - duration) <= self.groundtruth_epsilon:
                    groundtruth_result = True
            else:
                groundtruth_result = True
            return "time", duration, groundtruth_result
        else:
            return False
