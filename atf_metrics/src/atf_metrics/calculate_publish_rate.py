#!/usr/bin/env python
import copy
import math
import rospy

from atf_core import ATFError, ATFAnalyserError
from atf_msgs.msg import MetricResult, KeyValue, DataStamped
from atf_metrics import metrics_helper

class CalculatePublishRateParamHandler:
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
            try:
                series_mode = metric["series_mode"]
            except (TypeError, KeyError):
                series_mode = None
            metrics.append(CalculatePublishRate(metric["topic"], groundtruth, groundtruth_epsilon, series_mode))
        return metrics

class CalculatePublishRate:
    def __init__(self, topic, groundtruth, groundtruth_epsilon, series_mode):
        self.name = 'publish_rate'
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        if topic.startswith("/"): # we need to use global topics because rostopic.get_topic_class(topic) can not handle non-global topics and recorder will always record global topics starting with "/"
            self.topic = topic
        else:
            self.topic = "/" + topic
        self.series_mode = series_mode
        self.series = []
        self.data = DataStamped()
        self.counter = 0
        self.start_time = None

    def start(self, status):
        self.start_time = status.stamp
        self.active = True
        self.started = True

    def stop(self, status):
        # finally trigger update once again to update self.series and self.data
        self.update(self.topic, rospy.AnyMsg, status.stamp)
        self.active = False
        self.finished = True

    def pause(self, status):
        # TODO: Implement pause time and counter calculation
        #FIXME: check rate calculation in case of pause (counter, start_time)
        pass

    def purge(self, status):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def update(self, topic, msg, t):
        # get data if testblock is active
        if self.active:
            if topic == self.topic:
                self.counter += 1
                self.data.stamp = t
                self.data.data = round(self.get_publish_rate(),6)
                self.series.append(copy.deepcopy(self.data))  # FIXME handle fixed rates

    def get_publish_rate(self):
        publish_rate = self.counter / (self.data.stamp - self.start_time).to_sec()
        return publish_rate

    def get_topics(self):
        return [self.topic]

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.series = []
        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            if self.series_mode != None:
                metric_result.series = self.series
            metric_result.data = self.series[-1] # take last element from self.series
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)

            # fill details as KeyValue messages
            details = []
            details.append(KeyValue("topic", self.topic))
            metric_result.details = details

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
