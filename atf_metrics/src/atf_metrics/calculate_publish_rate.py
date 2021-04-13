#!/usr/bin/env python
import math
import rospy

from atf_metrics.error import ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped, TestblockStatus
from atf_metrics import metrics_helper

class CalculatePublishRateParamHandler:
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
        metric_type = "publish_rate"
        unit = "Hz"

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
            mode = MetricResult.SPAN_MEAN
        try:
            series_mode = params["series_mode"]
        except (TypeError, KeyError):
            series_mode = None

        try:
            min_observation_time = params["min_observation_time"]
        except (TypeError, KeyError):
            min_observation_time = 1.0

        return CalculatePublishRate(metric_name, testblock_name, min_observation_time, params["topic"], groundtruth, mode, series_mode, unit)

class CalculatePublishRate:
    def __init__(self, name, testblock_name, min_observation_time, topic, groundtruth, mode, series_mode, unit):
        self.name = name
        self.testblock_name = testblock_name
        self.status = TestblockStatus()
        self.status.name = testblock_name
        self.groundtruth = groundtruth
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.unit = unit

        self.start_time = None
        self.min_observation_time = min_observation_time
        if topic.startswith("/"): # we need to use global topics because rostopic.get_topic_class(topic) can not handle non-global topics and recorder will always record global topics starting with "/"
            self.topic = topic
        else:
            self.topic = "/" + topic
        self.counter = 0

    def start(self, status):
        self.status = status
        self.start_time = status.stamp

    def stop(self, status):
        self.status = status
        # finally trigger calculation once again to update self.series
        self.calculate_publish_rate(status.stamp)

    def pause(self, status):
        # TODO: Implement pause time and counter calculation
        #FIXME: check rate calculation in case of pause (counter, start_time)
        pass

    def purge(self, status):
        # TODO: Implement purge as soon as pause is implemented
        pass

    def update(self, topic, msg, t):
        # get data if testblock is active
        if self.status.status == TestblockStatus.ACTIVE:
            if topic == self.topic:
                self.counter += 1

                # wait for min_observation_time before calculating publish_rate to avoid tiny observation times and thus high publish rates, e.g. shortly after start
                if (t - self.start_time).to_sec() > self.min_observation_time:
                    self.calculate_publish_rate(t)

    def calculate_publish_rate(self, stamp):
        data = DataStamped()
        data.stamp = stamp
        data.data = round(self.counter / (data.stamp - self.start_time).to_sec(),6)
        self.series.append(data)  # FIXME handle fixed rates

    def get_topics(self):
        return [self.topic]

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.unit = self.unit
        metric_result.mode = self.mode
        metric_result.status = self.status.status
        metric_result.series = []
        metric_result.groundtruth.available = self.groundtruth.available
        metric_result.groundtruth.data = self.groundtruth.data
        metric_result.groundtruth.epsilon = self.groundtruth.epsilon

        if self.status.status != TestblockStatus.SUCCEEDED:
            metric_result.groundtruth.result = Groundtruth.FAILED
            metric_result.groundtruth.error_message = metrics_helper.extract_error_message(self.status)
            return metric_result

        # check if result is available
        if len(self.series) == 0:
            # let the analyzer know that this test failed
            metric_result.groundtruth.result = Groundtruth.FAILED
            metric_result.groundtruth.error_message = "testblock %s stopped without result"%self.testblock_name
            return metric_result

        # at this point we're sure that any result is available

        # set series
        if self.series_mode != None:
            metric_result.series = self.series

        # calculate metric data
        [metric_result.data, metric_result.min, metric_result.max, metric_result.mean, metric_result.std] = metrics_helper.calculate_metric_data(metric_result.name, metric_result.mode, self.series)

        # fill details as KeyValue messages
        details = []
        details.append(KeyValue("topic", self.topic))
        metric_result.details = details

        # evaluate metric data
        if metric_result.groundtruth.available: # groundtruth available
            if math.fabs(metric_result.groundtruth.data - metric_result.data.data) <= metric_result.groundtruth.epsilon:
                metric_result.groundtruth.result = Groundtruth.SUCCEEDED
                metric_result.groundtruth.error_message = "all OK"
            else:
                metric_result.groundtruth.result = Groundtruth.FAILED
                metric_result.groundtruth.error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth.data, metric_result.groundtruth.epsilon)

        else: # groundtruth not available
            metric_result.groundtruth.result = Groundtruth.SUCCEEDED
            metric_result.groundtruth.error_message = "all OK (no groundtruth available)"

        return metric_result
