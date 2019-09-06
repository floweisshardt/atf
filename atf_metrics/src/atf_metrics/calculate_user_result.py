#!/usr/bin/env python
import rospy
import math

from atf_core import ATFError
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
                #rospy.logwarn_throttle(10, "No groundtruth parameters given, skipping groundtruth evaluation for metric 'user_result' in testblock '%s'"%testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateUserResult(testblock_name, groundtruth, groundtruth_epsilon))
        return metrics

class CalculateUserResult:
    def __init__(self, testblock_name, groundtruth, groundtruth_epsilon):
        """
        Class for collecting the the user result.
        """
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.testblock_name = testblock_name
        self.testblock_result = None

    def start(self, timestamp):
        if self.testblock_result != None:
            #raise ATFError("user_result should be None but is already set")
            print "WARN: user_result should be None but is already set for testblock %s"%self.testblock_name
        self.active = True
        self.started = True

    def stop(self, timestamp):
        if self.testblock_result == None:
            #raise ATFError("user_result is not set")
            print "WARN: user_result for testblock %s is not set"%self.testblock_name
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        # TODO: Implement pause
        pass

    def purge(self, timestamp):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def update(self, topic, msg, t):
        if topic == "/atf/user_result":
            if msg.name == self.testblock_name:
                self.testblock_result = msg

    def get_topics(self):
        return ["/atf/user_result"]

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = "user_result"
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
            if self.testblock_result == None:
                print "ERROR user result for testblock %s not set"%self.testblock_name
                metric_result.data = None
            elif len(self.testblock_result.results) > 1:
                print "ERROR multiple user results found for testblock %s"%self.testblock_name
                metric_result.data = None
            else:
                metric_result.data = self.testblock_result.results[0].data

            # fill details as KeyValue messages
            details = []
            metric_result.details = details

            # evaluate metric data
            if metric_result.data != None and metric_result.groundtruth != None and metric_result.groundtruth_epsilon != None:
                if math.fabs(metric_result.groundtruth - metric_result.data) <= metric_result.groundtruth_epsilon:
                    metric_result.groundtruth_result = True
                    metric_result.groundtruth_error_message = "all OK"
                else:
                    metric_result.groundtruth_result = False
                    metric_result.groundtruth_error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data, metric_result.groundtruth, metric_result.groundtruth_epsilon)
                    #print metric_result.groundtruth_error_message

        if metric_result.data == None:
            metric_result.groundtruth_result = False
            metric_result.groundtruth_error_message = "no result"

        #print "\nmetric_result:\n", metric_result
        return metric_result