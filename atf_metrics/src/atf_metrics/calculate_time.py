#!/usr/bin/env python
import rospy
import time


class CalculateTimeParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    @staticmethod
    def parse_parameter(params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """

        return CalculateTime()


class CalculateTime:
    def __init__(self):
        """
        Class for calculating the time between the trigger 'ACTIVATE' and 'FINISH' on the topic assigned to the
        testblock.
        """
        self.start_time = rospy.Time()
        self.stop_time = rospy.Time()
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
        if self.finished:
            return "time", round((self.stop_time.to_sec() - self.start_time.to_sec()), 3)
        else:
            return False
