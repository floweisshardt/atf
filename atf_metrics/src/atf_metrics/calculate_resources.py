#!/usr/bin/env python
import numpy
import rospy
from copy import deepcopy
from atf_msgs.msg import *


class CalculateResources:
    """
    Class for calculating the average resource workload and writing the current resource data.
    The resource data is sent over the topic "/testing/Resources".
    """
    def __init__(self, resources):
        """
        Constructor.

        @param resources: a list of dictionaries containing the names of the resources and a list with
        the names of the nodes. Example: {"cpu":[move_group]}
        @type  resources: dict
        """

        self.active = False
        self.resources = resources
        self.node_data = {}
        self.size_io = len(IO.__slots__)
        self.size_network = len(Network.__slots__)
        self.average_data = {}

        # Sort resources after nodes
        for resource in self.resources:
            for node in self.resources[resource]:
                if node not in self.node_data:
                    self.node_data[node] = {resource: []}
                elif resource not in self.node_data[node]:
                    self.node_data[node].update({resource: []})

        self.average_data = deepcopy(self.node_data)

        rospy.Subscriber("/testing/Resources", Resources, self.process_resource_data, queue_size=1)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False

    def process_resource_data(self, msg):
        if self.active:
            for node in msg.nodes:
                try:
                    for resource in self.node_data[node.node_name]:
                        if resource == "cpu":
                            self.node_data[node.node_name][resource].append(node.cpu)
                        elif resource == "mem":
                            self.node_data[node.node_name][resource].append(node.memory)
                        elif resource == "io":
                            if len(self.node_data[node.node_name][resource]) == 0:
                                for i in xrange(0, self.size_io):
                                    self.node_data[node.node_name][resource].append([])
                            self.node_data[node.node_name][resource][0].append(node.io.read_count)
                            self.node_data[node.node_name][resource][1].append(node.io.write_count)
                            self.node_data[node.node_name][resource][2].append(node.io.read_bytes)
                            self.node_data[node.node_name][resource][3].append(node.io.write_bytes)
                        elif resource == "network":
                            if len(self.node_data[node.node_name][resource]) == 0:
                                for i in xrange(0, self.size_network):
                                    self.node_data[node.node_name][resource].append([])
                            self.node_data[node.node_name][resource][0].append(node.network.bytes_sent)
                            self.node_data[node.node_name][resource][1].append(node.network.bytes_recv)
                            self.node_data[node.node_name][resource][2].append(node.network.packets_sent)
                            self.node_data[node.node_name][resource][3].append(node.network.packets_recv)
                            self.node_data[node.node_name][resource][4].append(node.network.errin)
                            self.node_data[node.node_name][resource][5].append(node.network.errout)
                            self.node_data[node.node_name][resource][6].append(node.network.dropin)
                            self.node_data[node.node_name][resource][7].append(node.network.dropout)
                except KeyError:
                    pass

    def get_result(self):

        for node in self.node_data:
            for res in self.node_data[node]:
                if res == "io" or res == "network":
                    for values in self.node_data[node][res]:
                        self.average_data[node][res].append(float(numpy.mean(values)))
                else:
                    self.average_data[node][res] = float(numpy.mean(self.node_data[node][res]))

        # print self.average_data
        # return ""
        return "resources", self.node_data, "average_ressources", self.average_data
