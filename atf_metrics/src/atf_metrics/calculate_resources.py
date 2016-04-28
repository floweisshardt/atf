#!/usr/bin/env python
import numpy
import rospy

from atf_msgs.msg import *


class CalculateResourcesParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    def parse_parameter(self, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        self.params = params

        metrics = CalculateResources(self.params)

        return metrics


class CalculateResources:
    def __init__(self, resources):
        """
        Class for calculating the average resource workload and writing the current resource data.
        The resource data is sent over the topic "/testing/Resources".
        :param resources: a dictionary containing the names of the resources and a list with the names of the nodes.
        Example: {"cpu":[move_group], "mem": [move_group]}
        :type  resources: dict
        """

        self.active = False
        self.resources = resources
        self.node_data = {}
        self.size_io = len(IO.__slots__)
        self.size_network = len(Network.__slots__)
        self.finished = False

        # Sort resources after nodes
        for resource in self.resources:
            for node in self.resources[resource]:
                if node not in self.node_data:
                    self.node_data[node] = {resource: {"data": [], "average": [], "min": [], "max": []}}
                elif resource not in self.node_data[node]:
                    self.node_data[node].update({resource: {"data": [], "average": [], "min": [], "max": []}})

        rospy.Subscriber("/atf/resources", Resources, self.process_resource_data, queue_size=1)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    def pause(self):
        self.active = False

    @staticmethod
    def purge():
        pass

    def process_resource_data(self, msg):
        if self.active:
            for node in msg.nodes:
                try:
                    for resource in self.node_data[node.node_name]:
                        if resource == "cpu":
                            self.node_data[node.node_name][resource]["data"].append(round(node.cpu, 2))
                        elif resource == "mem":
                            self.node_data[node.node_name][resource]["data"].append(round(node.memory, 2))
                        elif resource == "io":
                            if len(self.node_data[node.node_name][resource]["data"]) == 0:
                                for i in xrange(0, self.size_io):
                                    self.node_data[node.node_name][resource]["data"].append([])
                            self.node_data[node.node_name][resource]["data"][0].append(round(node.io.read_count, 2))
                            self.node_data[node.node_name][resource]["data"][1].append(round(node.io.write_count, 2))
                            self.node_data[node.node_name][resource]["data"][2].append(round(node.io.read_bytes, 2))
                            self.node_data[node.node_name][resource]["data"][3].append(round(node.io.write_bytes, 2))
                        elif resource == "network":
                            if len(self.node_data[node.node_name][resource]["data"]) == 0:
                                for i in xrange(0, self.size_network):
                                    self.node_data[node.node_name][resource]["data"].append([])
                            self.node_data[node.node_name][resource]["data"][0].append(round(node.network.bytes_sent,
                                                                                             2))
                            self.node_data[node.node_name][resource]["data"][1].append(round(node.network.bytes_recv,
                                                                                             2))
                            self.node_data[node.node_name][resource]["data"][2].append(round(node.network.packets_sent,
                                                                                             2))
                            self.node_data[node.node_name][resource]["data"][3].append(round(node.network.packets_recv,
                                                                                             2))
                            self.node_data[node.node_name][resource]["data"][4].append(round(node.network.errin, 2))
                            self.node_data[node.node_name][resource]["data"][5].append(round(node.network.errout, 2))
                            self.node_data[node.node_name][resource]["data"][6].append(round(node.network.dropin, 2))
                            self.node_data[node.node_name][resource]["data"][7].append(round(node.network.dropout, 2))
                except KeyError:
                    pass

    def get_result(self):
        if self.finished:
            for node in self.node_data:
                for res in self.node_data[node]:
                    if len(self.node_data[node][res]["data"]) != 0:
                        if res == "io" or res == "network":
                            for values in self.node_data[node][res]["data"]:
                                self.node_data[node][res]["average"].append(float(round(numpy.mean(values), 2)))
                                self.node_data[node][res]["min"].append(float(round(min(values), 2)))
                                self.node_data[node][res]["max"].append(float(round(max(values), 2)))
                        else:
                            self.node_data[node][res]["average"] = float(round(numpy.mean(self.node_data[node][res]
                                                                                          ["data"]), 2))
                            self.node_data[node][res]["min"] = float(round(min(self.node_data[node][res]["data"]), 2))
                            self.node_data[node][res]["max"] = float(round(max(self.node_data[node][res]["data"]), 2))
                    del self.node_data[node][res]["data"]

            return "resources", self.node_data
        else:
            return False
