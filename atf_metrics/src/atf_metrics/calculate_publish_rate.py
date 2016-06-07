#!/usr/bin/env python
import rospy
import math

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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'publish_rate' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculatePublishRate(metric["topic"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculatePublishRate:
    def __init__(self, topic, groundtruth, groundtruth_epsilon):

        self.active = False
        self.finished = False
        self.topic = topic
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.counter = 0
        self.start_time = None
        self.stop_time = None

        rospy.Subscriber(topic, rospy.AnyMsg, self.callback,
                         queue_size=1)

    def callback(self, msg):
        if self.active:
            self.counter += 1

    def start(self, timestamp):
        self.active = True
        self.start_time = timestamp

    def stop(self, timestamp):
        self.active = False
        self.stop_time = timestamp
        self.finished = True

    def pause(self, timestamp):
        # TODO: Implement pause time and counter calculation
        #FIXME: check rate calculation in case of pause (counter, start_time and stop_time)
        pass

    def purge(self, timestamp):
        pass

    def get_result(self):
        groundtruth_result = None
        details = {"topic": self.topic}
        if self.finished:
            data = round(self.counter / (self.stop_time - self.start_time).to_sec(), 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "publish_rate", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
