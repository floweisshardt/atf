#!/usr/bin/env python
import rospy
import math

from atf_core import ATFAnalyserError
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
            groundtruth.data = params["groundtruth"]
            groundtruth.epsilon = params["groundtruth_epsilon"]
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

        # check if user result is set at all
        if self.metric_result == None:
            #print "no user result set"
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = "no result"
            #print "EXIT 0", metric_result.data.data, metric_result.groundtruth.result, metric_result.groundtruth.error_message
            return metric_result

        # check if groundtruth is set via user result (not all default values anymore)
        if self.metric_result.groundtruth.result\
            or self.metric_result.groundtruth.available\
            or self.metric_result.groundtruth.error_message != ""\
            or self.metric_result.groundtruth.data != 0\
            or self.metric_result.groundtruth.epsilon != 0:
            
            #print "groundtruth data is set from user within atf application for testblock %s. Skipping groundtruth evaluation from test_config"%self.testblock_name

            # use data from user result
            metric_result = self.metric_result

            # overwrite user_result data with mandatory ATF filds
            metric_result.name = self.name
            metric_result.started = True
            metric_result.finished = True
            metric_result.groundtruth.available = True
            #print "EXIT 1", metric_result.data.data, metric_result.groundtruth.result, metric_result.groundtruth.error_message
            return metric_result
        
        #print "no groundtruth set via user_result", self.metric_result.groundtruth

        metric_result.groundtruth.available = self.groundtruth.available
        metric_result.groundtruth.data = self.groundtruth.data
        metric_result.groundtruth.epsilon = self.groundtruth.epsilon
        
        # assign default value
        metric_result.groundtruth.result = None
        metric_result.groundtruth.error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            # check if user has set any metric_result (not all default values anymore)
            if not self.metric_result.started\
                and not self.metric_result.finished\
                and len(self.metric_result.series) == 0\
                and self.metric_result.data.stamp == rospy.Time(0)\
                and self.metric_result.data.data == 0:

                # let the analyzer know that this test failed
                metric_result.groundtruth.result = False
                metric_result.groundtruth.error_message = "user result for testblock %s not set"%self.testblock_name
                #print "EXIT 2", metric_result.data.data, metric_result.groundtruth.result, metric_result.groundtruth.error_message
                return metric_result

            metric_result.data = self.metric_result.data

            # fill details as KeyValue messages
            metric_result.details = self.metric_result.details

            # evaluate metric data
            if not metric_result.groundtruth.available: # no groundtruth given
                metric_result.groundtruth.result = True
                metric_result.groundtruth.error_message = "all OK (no groundtruth available)"
            else: # groundtruth available
                if math.fabs(metric_result.groundtruth.data - metric_result.data.data) <= metric_result.groundtruth.epsilon:
                    metric_result.groundtruth.result = True
                    metric_result.groundtruth.error_message = "all OK"
                else:
                    metric_result.groundtruth.result = False
                    metric_result.groundtruth.error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth.data, metric_result.groundtruth.epsilon)

        else: # testblock did not start and/or finish
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = "no result"

        if metric_result.groundtruth.result == None:
            raise ATFAnalyserError("Analysing failed, metric result is None for metric '%s'."%metric_result.name)

        #print "EXIT 3", metric_result.data.data, metric_result.groundtruth.result, metric_result.groundtruth.error_message
        return metric_result
