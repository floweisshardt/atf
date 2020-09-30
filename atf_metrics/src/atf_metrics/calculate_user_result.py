#!/usr/bin/env python
import math
import rospy

from atf_core import ATFAnalyserError, ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue

class CalculateUserResultParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, metric_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metric_type = "user_result"

        split_name = metric_name.split("::")
        if len(split_name) != 2:
            raise ATFConfigurationError("no valid metric name for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
        if split_name[0] != metric_type:
            raise ATFConfigurationError("called invalid metric handle for metric '%s' in testblock '%s'." %(metric_name, testblock_name))

        if type(params) is not dict:
            rospy.logerr("metric config not a dictionary")
            raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s': %s" %(metric_name, testblock_name, str(params)))

        # check for optional parameters
        groundtruth = Groundtruth()
        try:
            groundtruth.data = params["groundtruth"]["data"]
            groundtruth.epsilon = params["groundtruth"]["epsilon"]
            groundtruth.available = True
        except (TypeError, KeyError):
            groundtruth.data = 0
            groundtruth.epsilon = 0
            groundtruth.available = False

        return CalculateUserResult(metric_name, testblock_name, groundtruth)

class CalculateUserResult:
    def __init__(self, name, testblock_name, groundtruth):
        """
        Class for collecting the the user result.
        """
        self.name = name
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
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

        if not self.started:
            error_message = "testblock %s never started"%self.testblock_name
            #print error_message
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = error_message
            return metric_result

        if not self.finished:
            error_message = "testblock %s never stopped"%self.testblock_name
            #print error_message
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = error_message
            return metric_result

        # check if stop was called
        if self.metric_result == None:
            error_message = "YOU SHOULD NOT END UP HERE: testblock %s stopped but metric_result is None"%self.testblock_name
            #print error_message
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = error_message
            return metric_result

        # check if result is available
        if self.metric_result.groundtruth.error_message.startswith("!!USER ERROR!!: no user result set"): # TODO use from global field (same as in atf.stop())
            # let the analyzer know that this test failed
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = "testblock %s stopped without user_result"%self.testblock_name
            return metric_result

        # at this point we're sure that any user_result is available

        # overwrite user_result data with mandatory ATF fields
        metric_result = self.metric_result
        metric_result.name = self.name
        metric_result.started = True
        metric_result.finished = True

        # evaluate metric data
        if self.groundtruth.available: # groundtruth available
            # overwrite grundtruth with data from yaml file
            metric_result.groundtruth = self.groundtruth
            if math.fabs(metric_result.groundtruth.data - metric_result.data.data) <= metric_result.groundtruth.epsilon:
                metric_result.groundtruth.result = True
                metric_result.groundtruth.error_message = "all OK"
            else:
                metric_result.groundtruth.result = False
                metric_result.groundtruth.error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth.data, metric_result.groundtruth.epsilon)

        else: # groundtruth not available
            metric_result.groundtruth.result = True
            metric_result.groundtruth.error_message = "all OK (no groundtruth available)"

        return metric_result
