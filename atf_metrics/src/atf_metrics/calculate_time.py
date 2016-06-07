#!/usr/bin/env python
import rospy
import math

class CalculateTimeParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []

        # special case for time (can have empty list of parameters, thus adding dummy groundtruth information)
        if params == []:
            params.append({"groundtruth":None, "groundtruth_epsilon":None})

        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'time' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateTime(groundtruth, groundtruth_epsilon))
        return metrics

class CalculateTime:
    def __init__(self, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the time between the trigger 'ACTIVATE' and 'FINISH' on the topic assigned to the
        testblock.
        """
        self.start_time = None
        self.stop_time = None
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False

    def start(self, timestamp):
        self.start_time = timestamp

    def stop(self, timestamp):
        self.stop_time = timestamp
        self.finished = True

    def pause(self, timestamp):
        # TODO: Implement pause time calculation
        pass

    def purge(self, timestamp):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def get_result(self):
        groundtruth_result = None
        details = None
        if self.finished:
            data = round((self.stop_time - self.start_time).to_sec(), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "time", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
