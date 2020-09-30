#!/usr/bin/env python
import copy
import math
import rospy

from atf_core import ATFAnalyserError, ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped
from atf_metrics import metrics_helper

class CalculateTimeParamHandler:
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
        metric_type = "time"

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
        try:
            mode = params["mode"]
        except (TypeError, KeyError):
            mode = MetricResult.SNAP
        try:
            series_mode = params["series_mode"]
        except (TypeError, KeyError):
            series_mode = None

        return CalculateTime(metric_name, testblock_name, groundtruth, mode, series_mode)

class CalculateTime:
    def __init__(self, name, testblock_name, groundtruth, mode, series_mode):
        """
        Class for calculating the time between the trigger 'ACTIVATE' and 'FINISH' on the topic assigned to the
        testblock.
        """
        self.name = name
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.testblock_name = testblock_name
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.data = DataStamped()
        self.start_time = None

    def start(self, status):
        self.start_time = status.stamp
        self.active = True
        self.started = True

    def stop(self, status):
        self.data.stamp = status.stamp
        self.data.data = round((self.data.stamp - self.start_time).to_sec(), 6)
        self.series.append(copy.deepcopy(self.data))  # FIXME handle fixed rates
        self.active = False
        self.finished = True

    def pause(self, status):
        # TODO: Implement pause time calculation
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
        metric_result.mode = self.mode
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.series = []
        metric_result.groundtruth.available = self.groundtruth.available
        metric_result.groundtruth.data = self.groundtruth.data
        metric_result.groundtruth.epsilon = self.groundtruth.epsilon

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

        # check if result is available
        if len(self.series) == 0:
            # let the analyzer know that this test failed
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = "testblock %s stopped without result"%self.testblock_name
            return metric_result

        # at this point we're sure that any result is available

        # calculate metric data
        if self.series_mode != None:
            metric_result.series = self.series
        if metric_result.mode == MetricResult.SNAP:
            metric_result.data = self.series[-1]                           # take last element from self.series for data and stamp
            metric_result.min = metric_result.data
            metric_result.max = metric_result.data
            metric_result.mean = metric_result.data.data
            metric_result.std = 0.0
        elif metric_result.mode == MetricResult.SPAN_MEAN:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data.data = metric_result.mean                   # take mean for data
            metric_result.data.stamp = self.series[-1].stamp               # take stamp from last element in self.series for stamp
        elif metric_result.mode == MetricResult.SPAN_MIN:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.min
        elif metric_result.mode == MetricResult.SPAN_ABSMIN:
            metric_result.min = metrics_helper.get_absmin(self.series)
            metric_result.max = metrics_helper.get_absmax(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.min
        elif metric_result.mode == MetricResult.SPAN_MAX:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.max
        elif metric_result.mode == MetricResult.SPAN_ABSMAX:
            metric_result.min = metrics_helper.get_absmin(self.series)
            metric_result.max = metrics_helper.get_absmax(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.max
        else: # invalid mode
            raise ATFAnalyserError("Analysing failed, invalid mode '%s' for metric '%s'."%(metric_result.mode, metric_result.name))

        # fill details as KeyValue messages
        details = []
        details.append(KeyValue("start_time", str(self.start_time.to_sec())))
        details.append(KeyValue("stop_time", str(self.data.data)))
        metric_result.details = details

        # evaluate metric data
        if metric_result.groundtruth.available: # groundtruth available
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
