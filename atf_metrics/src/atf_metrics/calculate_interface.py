#!/usr/bin/env python
import fnmatch
import math
import re
import rospy

from atf_metrics.error import ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped, TestblockStatus
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
        unit = "%"

        split_name = metric_name.split("::")
        if len(split_name) != 2:
            raise ATFConfigurationError("no valid metric name for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
        if split_name[0] != metric_type:
            raise ATFConfigurationError("called invalid metric handle for metric '%s' in testblock '%s'." %(metric_name, testblock_name))

        if type(params) is not dict:
            rospy.logerr("metric config not a dictionary")
            raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s': %s" %(metric_name, testblock_name, str(params)))

        for interface, data in list(params.items()):
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

        return CalculateInterface(metric_name, testblock_name, params, mode, series_mode, unit)

class CalculateInterface:
    def __init__(self, name, testblock_name, params, mode, series_mode, unit):
        """
        Class for calculating the interface type.
        """
        self.name = name
        self.testblock_name = testblock_name
        self.status = TestblockStatus()
        self.groundtruth = Groundtruth()
        self.groundtruth.available = True
        self.groundtruth.data = 100         # this is the max score
        self.groundtruth.epsilon = 0        # no deviation from max score allowed
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.unit = unit

        self.interface_details = {}
        self.params = params

    def start(self, status):
        self.status = status

    def stop(self, status):
        self.status = status

    def pause(self, status):
        pass

    def purge(self, status):
        pass

    def update(self, topic, msg, t):
        # get data if testblock is active
        if self.status.status == TestblockStatus.ACTIVE:
            if topic == "/atf/api" and msg.testblock_name == self.testblock_name:
                api_dict = self.msg_to_dict(msg)
                interface_data, self.interface_details = self.calculate_data_and_details(api_dict)
                data = DataStamped()
                data.stamp = t
                data.data = interface_data
                self.series.append(data)  # FIXME handle fixed rates

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
        regex = fnmatch.translate(name)
        reobj = re.compile(regex)
        for i in interface:
            if reobj.match(i[0]):
                if i[1] == topic_type:
                    return True, True # all OK
                else:
                    return True, False # only name OK, but type failed
        return False, False

    def get_topics(self):
        topics = []
        #print "self.metric=", self.metric
        if "publishers" in list(self.params.keys()):
            for topic_with_type in self.params["publishers"]:
                topic = topic_with_type[0]
                if topic not in topics:
                    topics.append(topic)
        if "subscribers" in list(self.params.keys()):
            for topic_with_type in self.params["subscribers"]:
                topic = topic_with_type[0]
                if topic not in topics:
                    topics.append(topic)
        return topics

    def calculate_data_and_details(self, api_dict):
        # As interface metric is not numeric, we'll use the following numeric representation:
        # max score = 100.0
        # node in api: score = 33.3
        # all interfaces available: score = 66.6
        # all types correct: score = 100.0
        data = None
        groundtruth = Groundtruth()
        details = None
        node_name = self.params['node']

        if node_name not in api_dict:
            details = "node " + node_name + " is not in api"
            groundtruth.result = Groundtruth.FAILED
            data = 0.0
            return data, details

        details = "node " + node_name + " is in api"
        groundtruth.result = Groundtruth.SUCCEEDED
        data = 100.0
        for interface, interface_data in list(self.params.items()):
            if interface == "publishers" or interface == "subscribers" or interface == "services":
                for topic_name, topic_type in interface_data:
                    #print "node_name=", node_name
                    #print "topic_name=", topic_name
                    #print "topic_type=", topic_type
                    #print "api_dict[node_name][interface]=", api_dict[node_name][interface]
                    name_check, type_check = self.check_interface(topic_name, topic_type, api_dict[node_name][interface])
                    if not name_check:
                        details += ", but " + topic_name + " is not in the list of " + interface + " of node " + node_name + ". " + interface + " are: " + str(api_dict[node_name][interface])
                        groundtruth.result = Groundtruth.FAILED
                        data = min(data, 33.3)
                    else:
                        if not type_check:
                            details += ", but " + topic_name + " (of type " + topic_type + ") is not in the list of " + interface + " of node " + node_name + ". " + interface + " are: " + str(api_dict[node_name][interface])
                            groundtruth.result = Groundtruth.FAILED
                            data = min(data, 66.6)
        return data, details

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
        details.append(KeyValue("api_status", self.interface_details))
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
