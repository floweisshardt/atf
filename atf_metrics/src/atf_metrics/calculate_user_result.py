#!/usr/bin/env python
import rospy
import math

from atf_core import ATFAnalyserError
from atf_msgs.msg import MetricResult, KeyValue

class CalculateUserResultParamHandler:
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

        # special case for user_result (can have empty list of parameters, thus adding dummy groundtruth information)
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
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateUserResult(testblock_name, groundtruth, groundtruth_epsilon))
        return metrics

class CalculateUserResult:
    def __init__(self, testblock_name, groundtruth, groundtruth_epsilon):
        """
        Class for collecting the the user result.
        """
        self.name = 'user_result'
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.testblock_name = testblock_name
        self.metric_result = None

    def start(self, status):
        if self.metric_result != None:
            print "WARN: user_result should be None but is already set for testblock %s"%self.testblock_name
        self.active = True
        self.started = True

    def stop(self, status):
        if self.metric_result != None:
            print "WARN: user_result should be None but is already set for testblock %s"%self.testblock_name
        self.metric_result = status.user_result
        self.active = False
        self.finished = True

    def pause(self, status):
        # TODO: Implement pause
        pass

    def purge(self, status):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def update(self, topic, msg, t):
        pass

    def get_topics(self):
        return []

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove

        # check if user result is set
        if self.metric_result != None and not (self.metric_result.groundtruth_result == False\
            and self.metric_result.groundtruth_error_message == ""\
            and self.metric_result.groundtruth == 0.0\
            and self.metric_result.groundtruth_epsilon == 0.0):
            #print "groundtruth data is set from user within atf application for testblock %s. Skipping groundtruth evaluation from test_config"%self.testblock_name
            # use data from user result
            metric_result = self.metric_result
            return metric_result

        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            if self.metric_result == None:
                print "ERROR user result for testblock %s not set"%self.testblock_name
                metric_result.data = None
            else:
                metric_result.data = self.metric_result.data

            # fill details as KeyValue messages
            metric_result.details = self.metric_result.details

            # evaluate metric data
            if metric_result.groundtruth == None and metric_result.groundtruth_epsilon == None: # no groundtruth given
                metric_result.groundtruth_result = True
                metric_result.groundtruth_error_message = "all OK (no groundtruth available)"
            elif metric_result.data != None and metric_result.groundtruth != None and metric_result.groundtruth_epsilon != None:
                if math.fabs(metric_result.groundtruth - metric_result.data.data) <= metric_result.groundtruth_epsilon:
                    metric_result.groundtruth_result = True
                    metric_result.groundtruth_error_message = "all OK"
                else:
                    metric_result.groundtruth_result = False
                    metric_result.groundtruth_error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth, metric_result.groundtruth_epsilon)
            else:
                metric_result.groundtruth_result = False
                metric_result.groundtruth_error_message = "metric evaluation failed"

        if metric_result.data == None:
            metric_result.groundtruth_result = False
            metric_result.groundtruth_error_message = "no result"

        if metric_result.groundtruth_result == None:
            raise ATFAnalyserError("Analysing failed, metric result is None for metric '%s'."%metric_result.name)

        return metric_result
