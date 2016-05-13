#!/usr/bin/env python
import rospy
import math

class CalculateTimeParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []

        # special case for time (can have no parameters and thus is no list)
        if params == None:
            metrics.append(CalculateTime(None, None))
            return metrics
        
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'time'")
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
        self.start_time = rospy.Time()
        self.stop_time = rospy.Time()
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False

    def start(self):
        self.start_time = rospy.Time.now()

    def stop(self):
        self.stop_time = rospy.Time.now()
        self.finished = True

    def pause():
        # TODO: Implement pause time calculation
        pass

    def purge():
        pass

    def get_result(self):
        groundtruth_result = None
        details = None
        if self.finished:
            data = round(self.stop_time.to_sec() - self.start_time.to_sec(), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "time", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
