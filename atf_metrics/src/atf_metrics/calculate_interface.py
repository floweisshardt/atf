#!/usr/bin/env python
import rospy
from atf_msgs.msg import Api

class CalculateInterfaceParamHandler:
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
            for interface, data in metric.items():
                if type(data) is list:
                    new_data = []
                    for interface_name, interface_type in data:
                        if interface_name[0] != "/":
                            interface_name = "/" + interface_name
                        new_data.append([interface_name, interface_type])
                    metric[interface] = new_data
                elif type(data) is str:
                    if data[0] != "/":
                        metric[interface] = "/" + data
            metrics.append(CalculateInterface(testblock_name, metric))
        return metrics

class CalculateInterface:
    def __init__(self, testblock_name, metric):
        """
        Class for calculating the interface type.
        """
        self.finished = False
        self.api_dict = {}
        self.testblock_name = testblock_name
        self.metric = metric
        rospy.Subscriber("/atf/" + self.testblock_name + "/api", Api, self.api_callback)

    def api_callback(self, msg):
        self.api_dict = self.msg_to_dict(msg)

    def start(self):
        pass

    def stop(self):
        self.finished = True

    def pause(self):
        pass

    def purge(self):
        pass

    def msg_to_dict(self, msg):
        api_dict = {}
        
        for node_api in msg.nodes:
            #print "node_api=", node_api
            api_dict[node_api.name] = {}
            api_dict[node_api.name]["publishers"] = node_api.interface.publishers
            api_dict[node_api.name]["subscribers"] = node_api.interface.subscribers
            api_dict[node_api.name]["services"] = node_api.interface.services
            #TODO actions
        return api_dict

    def get_result(self):
        data = 0 # interface metric is not numeric
        groundtruth_result = True # interface metric not usable without groundtruth
        groundtruth = None
        groundtruth_epsilon = None
        details = None #self.api_dict

        #print "self.metric=", self.metric
        #print "self.api_dict=", self.api_dict

        if self.metric['node'] not in self.api_dict:
            print "node", self.metric['node'], "is NOT in api"
            groundtruth_result = False
            #data = {"nodes": self.api_dict.keys()}
            #groundtruth = {"node": self.metric['node']}
        else:
            print "node", self.metric['node'], "is in api"
            for interface, interface_data in self.metric.items():
                if interface != "node":
                    for topic_name, topic_type in interface_data:
                        #print "topic_name=", topic_name
                        #print "topic_type=", topic_type
                        #print "self.api_dict[self.metric['node']][interface]=", self.api_dict[self.metric['node']][interface]
                        if topic_name not in self.api_dict[self.metric['node']][interface]:
                            print topic_name, "is NOT an interface of node", self.metric['node'], "interfaces:", self.api_dict[self.metric['node']][interface]
                            groundtruth_result = False
                            #data = {interface: self.api_dict[self.metric['node']][interface]}
                            #groundtruth = {interface: self.metric[interface]}
        if self.finished:
            return "interface", data, groundtruth_result, groundtruth, groundtruth_epsilon, details
        else:
            return False
