#!/usr/bin/env python
import copy
import math
import rospy

from atf_core import ATFAnalyserError, ATFConfigurationError
from atf_msgs.msg import Api
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped
from atf_metrics import metrics_helper

class CalculateInterfaceParamHandler:
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
        metric_type = "interface"

        split_name = metric_name.split("::")
        if len(split_name) != 2:
            raise ATFConfigurationError("no valid metric name for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
        if split_name[0] != metric_type:
            raise ATFConfigurationError("called invalid metric handle for metric '%s' in testblock '%s'." %(metric_name, testblock_name))

        if type(params) is not dict:
            rospy.logerr("metric config not a dictionary")
            raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s': %s" %(metric_name, testblock_name, str(params)))

        for interface, data in params.items():
            if type(data) is list:
                new_data = []
                for interface_name, interface_type in data:
                    if interface_name[0] != "/":
                        interface_name = "/" + interface_name
                    new_data.append([interface_name, interface_type])
                params[interface] = new_data
            elif type(data) is str:
                if data[0] != "/":
                    params[interface] = "/" + data
        # check for optional parameters
        try:
            mode = params["mode"]
        except (TypeError, KeyError):
            mode = MetricResult.SNAP
        try:
            series_mode = params["series_mode"]
        except (TypeError, KeyError):
            series_mode = None

        return CalculateInterface(metric_name, testblock_name, params, mode, series_mode)

class CalculateInterface:
    def __init__(self, name, testblock_name, params, mode, series_mode):
        """
        Class for calculating the interface type.
        """
        self.name = name
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = Groundtruth()
        self.groundtruth.available = True
        self.groundtruth.data = 100         # this is the max score
        self.groundtruth.epsilon = 0        # no deviation from max score allowed
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.data = DataStamped()
        self.interface_details = {}
        self.api_dict = {}
        self.testblock_name = testblock_name
        self.params = params

    def start(self, status):
        self.active = True
        self.started = True

    def stop(self, status):
        self.active = False
        self.finished = True

    def pause(self, status):
        pass

    def purge(self, status):
        pass

    def update(self, topic, msg, t):
        # get data if testblock is active
        if self.active:
            if topic == "/atf/api" and msg.testblock_name == self.testblock_name:
                self.api_dict = self.msg_to_dict(msg)
                interface_data, self.interface_details = self.calculate_data_and_details()
                self.data.stamp = t
                self.data.data = interface_data
                self.series.append(copy.deepcopy(self.data))  # FIXME handle fixed rates

    def msg_to_dict(self, msg):
        api_dict = {}

        for node_api in msg.nodes:
            #print "node_api=", node_api
            api_dict[node_api.name] = {}

            api_dict[node_api.name]["publishers"] = []
            for item in node_api.interface.publishers:
                api_dict[node_api.name]["publishers"].append([item.name, item.type])

            api_dict[node_api.name]["subscribers"] = []
            for item in node_api.interface.subscribers:
                api_dict[node_api.name]["subscribers"].append([item.name, item.type])

            api_dict[node_api.name]["services"] = []
            for item in node_api.interface.services:
                api_dict[node_api.name]["services"].append([item.name, item.type])

            #TODO actions
        #print "api_dict=", api_dict
        return api_dict

    def check_interface(self, name, topic_type, interface):
        #print "name=", name
        #print "topic_type=", topic_type
        #print "interface=", interface
        for i in interface:
            if i[0] == name:
                if i[1] == topic_type:
                    return True, True # all OK
                else:
                    return True, False # only name OK, but type failed
        return False, False

    def get_topics(self):
        topics = []
        #print "self.metric=", self.metric
        if "publishers" in self.params.keys():
            for topic_with_type in self.params["publishers"]:
                topic = topic_with_type[0]
                if topic not in topics:
                    topics.append(topic)
        if "subscribers" in self.params.keys():
            for topic_with_type in self.params["subscribers"]:
                topic = topic_with_type[0]
                if topic not in topics:
                    topics.append(topic)
        return topics

    def calculate_data_and_details(self):
        # As interface metric is not numeric, we'll use the following numeric representation:
        # max score = 100.0
        # node in api: score = 33.3
        # all interfaces available: score = 66.6
        # all types correct: score = 100.0
        data = None
        groundtruth = Groundtruth()
        details = None
        node_name = self.params['node']
        if node_name not in self.api_dict:
            details = "node " + node_name + " is not in api"
            groundtruth.result = False
            data = 0.0
        else:
            details = "node " + node_name + " is in api"
            groundtruth.result = True
            data = 100.0
            for interface, interface_data in self.params.items():
                if interface == "publishers" or interface == "subscribers" or interface == "services":
                    for topic_name, topic_type in interface_data:
                        #print "node_name=", node_name
                        #print "topic_name=", topic_name
                        #print "topic_type=", topic_type
                        #print "self.api_dict[node_name][interface]=", self.api_dict[node_name][interface]
                        name_check, type_check = self.check_interface(topic_name, topic_type, self.api_dict[node_name][interface])
                        if not name_check:
                            details += ", but " + topic_name + " is not an interface of node " + node_name + ". Interfaces are: " + str(self.api_dict[node_name][interface])
                            groundtruth.result = False
                            data = 33.3
                        else:
                            if not type_check:
                                details += ", but " + topic_name + " (with type " + topic_type + ") is not an interface of node " + node_name + ". Interfaces are: " + str(self.api_dict[node_name][interface])
                                groundtruth.result = False
                                data = 66.0
                            else: # all Ok
                                details += ", all interfaces of node " + node_name + ": OK"
                                groundtruth.result = True
                                data = 100.0
        return data, details

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
        
        # assign default value
        metric_result.groundtruth.result = None
        metric_result.groundtruth.error_message = None

        if metric_result.started and metric_result.finished and len(self.series) != 0: #  we check if the testblock was ever started and stopped and if result data is available
            # calculate metric data
            if self.series_mode != None:
                metric_result.series = self.series
            if metric_result.mode == MetricResult.SNAP:
                metric_result.data = self.series[-1]                           # take last element from self.series for data and stamp
                metric_result.min = metric_result.data
                metric_result.max = metric_result.data
                metric_result.mean = metric_result.data.data
                metric_result.std = 0.0
            elif metric_result.mode == MetricResult.SPAN:
                metric_result.data.data = metrics_helper.get_mean(self.series) # take mean for data
                metric_result.data.stamp = self.series[-1].stamp               # take stamp from last element in self.series for stamp
                metric_result.min = metrics_helper.get_min(self.series)
                metric_result.max = metrics_helper.get_max(self.series)
                metric_result.mean = metrics_helper.get_mean(self.series)
                metric_result.std = metrics_helper.get_std(self.series)
            else: # invalid mode
                raise ATFAnalyserError("Analysing failed, invalid mode '%s' for metric '%s'."%(metric_result.mode, metric_result.name))

            # fill details as KeyValue messages
            details = []
            details.append(KeyValue("api_status", self.interface_details))
            metric_result.details = details

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

        return metric_result
