#!/usr/bin/env python
import time
import rospy
import rostopic


class CalculatePublishRateParamHandler:
    def __init__(self):
        pass

    def parse_parameter(self, params):
        print "params=", params

        if type(params) is list:
            metrics = []
            for metric in params:
                metrics.append(CalculatePublishRate(metric['name'], metric['interface_type'], metric['msg_type'], metric['groundtruth'], metric['groundtruth_epsilon']))
            return metrics
        else:
            return CalculatePublishRate(metric['name'], metric['interface_type'], metric['msg_type'], metric['groundtruth'], metric['groundtruth_epsilon'])


class CalculatePublishRate:
    def __init__(self, name, interface_type, msg_type):

        self.active = False
        self.finished = False
        self.hz = hz
        self.error = error
        self.topic = topic
        self.counter = 0
        self.start_time = 0
        self.stop_time = 0

        rospy.Subscriber(topic, rostopic.get_topic_class(self.topic, blocking=True)[0], self.message_callback,
                         queue_size=1)

    def message_callback(self, msg):
        if self.active:
            self.counter += 1

    def start(self):
        self.active = True
        self.start_time = time.time()

    def stop(self):
        self.active = False
        self.stop_time = time.time()
        self.finished = True

    def pause(self):
        self.active = False

    @staticmethod
    def purge():
        pass

    def get_result(self):
        if self.finished:
            rate = self.counter / (self.stop_time - self.start_time)
            result = 0
            if (rate - self.hz) > self.error or (self.hz - rate) > self.error:
                result = rate - self.hz
            #return "publish_rate", {self.topic: round(rate, 3)}
            return "publish_rate", {self.topic: round(result, 3)}
        else:
            return False
