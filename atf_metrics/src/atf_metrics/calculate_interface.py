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

    def get_result(self):
        data = 0 # interface metric is not numeric
        groundtruth_result = True # interface metric not usable without groundtruth
        groundtruth = None
        groundtruth_epsilon = None
        details = None #self.api_dict

        #print "self.metric=", self.metric
        #print "self.api_dict=", self.api_dict

        node_name = self.metric['node']
        if node_name not in self.api_dict:
            print "node", node_name, "is NOT in api"
            groundtruth_result = False
            #data = {"nodes": self.api_dict.keys()}
            #groundtruth = {"node": node_name}
        else:
            print "node", node_name, "is in api"
            for interface, interface_data in self.metric.items():
                if interface != "node":
                    for topic_name, topic_type in interface_data:
                        #print ""
                        #print "node_name=", node_name
                        #print "topic_name=", topic_name
                        #print "topic_type=", topic_type
                        #print "self.api_dict[node_name][interface]=", self.api_dict[node_name][interface]
                        if not self.is_name_in_interface(topic_name, topic_type, self.api_dict[node_name][interface]):
                            print "  but", topic_name, "(with type", topic_type ,") is NOT an interface of node", node_name, "interfaces are:", self.api_dict[node_name][interface]
                            groundtruth_result = False
                            #data = {interface: self.api_dict[node_name][interface]}
                            #groundtruth = {interface: self.metric[interface]}
        if self.finished:
            return "interface", data, groundtruth_result, groundtruth, groundtruth_epsilon, details
        else:
            return False
    
    def is_name_in_interface(self, name, topic_type, interface):
        #print "name=", name
        #print "topic_type=", topic_type
        #print "interface=", interface
        for i in interface:
            if i[0] == name and i[1] == topic_type:
                return True
        return False
