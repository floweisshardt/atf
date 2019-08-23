#!/usr/bin/env python
import rospy
import math

from atf_msgs.msg import MetricResult, KeyValue

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
                #rospy.logwarn_throttle(10, "No groundtruth parameters given, skipping groundtruth evaluation for metric 'time' in testblock '%s'"%testblock_name)
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
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.start_time = None
        self.stop_time = None

    def start(self, timestamp):
        self.start_time = timestamp
        self.active = True
        self.started = True

    def stop(self, timestamp):
        self.stop_time = timestamp
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        # TODO: Implement pause time calculation
        pass

    def purge(self, timestamp):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def update(self, topic, msg, t):
        pass

    def get_topics(self):
            return []

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = "time"
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            metric_result.data = round((self.stop_time - self.start_time).to_sec(), 3)

            # fill details as KeyValue messages
            details = []
            metric_result.details = details

            # evaluate metric data
            if metric_result.groundtruth != None and metric_result.groundtruth_epsilon != None:
                if math.fabs(metric_result.groundtruth - metric_result.data) <= metric_result.groundtruth_epsilon:
                    metric_result.groundtruth_result = True
                    metric_result.groundtruth_error_message = "all OK"
                else:
                    metric_result.groundtruth_result = False
                    metric_result.groundtruth_error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data, metric_result.groundtruth, metric_result.groundtruth_epsilon)
                    #print metric_result.groundtruth_error_message

        if metric_result.data == None:
            raise ATFAnalyserError("Analysing failed, no metric result available for metric '%s'."%metric_result.name)

        #print "\nmetric_result:\n", metric_result
        return metric_result