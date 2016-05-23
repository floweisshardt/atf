#!/usr/bin/env python
import rospy
import sys
from rosapi.srv import Nodes, Topics
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
        #print "params=", params
        metrics = []
        
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            #print "metric1=", metric
            for interface, data in metric.items():
                #print "interface=", interface
                #print "data=", data
                if type(data) is list:
                    new_data = []
                    for interface_name, interface_type in data:
                        #print "interface_name=", interface_name
                        #print "interface_type=", interface_type
                        if interface_name[0] != "/":
                            interface_name = "/" + interface_name
                        new_data.append([interface_name, interface_type])
                    metric[interface] = new_data
                elif type(data) is str:
                    if data[0] != "/":
                        metric[interface] = "/" + data
            #print "metric2=", metric
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
        #print "got api callback in ", self.testblock_name, ":", msg
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
        
        #print "msg=", msg
        for node_api in msg.nodes:
            #print "node_api=", node_api
            api_dict[node_api.name] = {}
            api_dict[node_api.name]["publishers"] = node_api.interface.publishers
            api_dict[node_api.name]["subscribers"] = node_api.interface.subscribers
            api_dict[node_api.name]["services"] = node_api.interface.services
            #TODO actions
        #print "api_dict=", api_dict
        return api_dict

    def get_result(self):
        data = None
        groundtruth_result = None
        groundtruth = None
        groundtruth_epsilon = None
        details = {}

        #print "self.metric=", self.metric
        #print "self.api_dict=", self.api_dict
        
        if self.metric['node'] not in self.api_dict:
            print "node", self.metric['node'], "is NOT in api"
            groundtruth_result = False
            data = {"nodes": self.api_dict.keys()}
            groundtruth = {"node": self.metric['node']}
        else:
            print "node", self.metric['node'], "is in api"
            for interface, data in self.metric.items():
                if interface != "node":
                    for topic_name, topic_type in data:
                        #print "topic_name=", topic_name
                        #print "topic_type=", topic_type
                        if topic_name not in self.api_dict[self.metric['node']][interface]:
                            print topic_name, "is NOT an interface of node", self.metric['node'], "interfaces:", self.api_dict[self.metric['node']][interface]
                            groundtruth_result = False
                            data = {interface: self.api_dict[self.metric['node']][interface]}
                            groundtruth = {interface: self.metric[interface]}
        details = self.api_dict
        if self.finished:
            return "interface", data, groundtruth_result, groundtruth, groundtruth_epsilon, details
        else:
            return False
