#!/usr/bin/env python
import numpy
import rospy
import math

from atf_msgs.msg import Resources, IO, Network


class CalculateResourcesMemParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        if not isinstance(params, list):
            rospy.logerr("metric config not a list")
            return False
        metrics = []
        print "params:", params
        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
                print "groundtruth", groundtruth, "groundtruth_epsilon", groundtruth_epsilon
                if 'groundtruth' in metric:
                    del metric['groundtruth']
                if 'groundtruth_epsilon' in metric:
                    del metric['groundtruth_epsilon']
            except (TypeError, KeyError):
                rospy.logwarn(
                    "No groundtruth parameters given, skipping groundtruth evaluation for metric 'resources' in testblock '%s'",
                    testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            print "metric:", metric
            metrics.append(CalculateResourcesMem(metric["nodes"], groundtruth, groundtruth_epsilon))
        return metrics


class CalculateResourcesMem:
    def __init__(self, nodes, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the average resource workload and writing the current resource data.
        The resource data is sent over the topic "/testing/Resources".
        :param resources: a dictionary containing the names of the resources and a list with the names of the nodes.
        Example: {"cpu":[move_group], "mem": [move_group]}
        :type  resources: dict
        """

        self.active = False
        self.resource = "mem"
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.node_data = {}
        self.size_io = len(IO.__slots__)
        self.size_network = len(Network.__slots__)
        self.finished = False

        # Sort resources after nodes
        print "node data:", self.node_data
        for node in nodes:
            if node not in self.node_data:
                print "node : ", node
                self.node_data[node] = {self.resource: {"data": [], "average": [], "min": [], "max": []}}
            # elif resource not in self.node_data[node]:
            #     self.node_data[node].update({resource: {"data": [], "average": [], "min": [], "max": []}})
        print "node data after:", self.node_data
        rospy.Subscriber("/atf/resources", Resources, self.process_resource_data, queue_size=1)

    def start(self, timestamp):
        self.active = True

    def stop(self, timestamp):
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        self.active = False

    def purge(self, timestamp):
        pass

    def process_resource_data(self, msg):
        #print "--------------------------------------\nprocess data \n msg:", msg, "\n active", self.active
        if self.active:
            for node in msg.nodes:
                try:
                    for resource in self.node_data[node.node_name]:
                        #print "nodes:", msg.nodes, "\n node data:", self.node_data, "\n resource", resource
                        if resource == "mem":
                            self.node_data[node.node_name][resource]["data"].append(round(node.memory, 2))
                except KeyError:
                    pass

    def get_result(self):
        groundtruth_result = None
        details = {"sum of nodes":[]}
        average_sum = 0.0

        if self.finished:
            #print "----------------------------- \n node data:", self.node_data
            for node in self.node_data:
                #print " node:", node
                for res in self.node_data[node]:
                    #print "res", res
                    if len(self.node_data[node][res]["data"]) != 0:
                        self.node_data[node][res]["average"] = float(round(numpy.mean(self.node_data[node][res]
                                                                                          ["data"]), 2))
                        self.node_data[node][res]["min"] = float(round(min(self.node_data[node][res]["data"]), 2))
                        self.node_data[node][res]["max"] = float(round(max(self.node_data[node][res]["data"]), 2))
                        average_sum += float(round(numpy.mean(self.node_data[node][res]["data"]), 2))
                        print "average sum:", average_sum
                    del self.node_data[node][res]["data"]

                    details["sum of nodes"].append(node)
                    # details["nodes"][node].append({"max":self.node_data[node][res]["max"]})
                    # details["nodes"][node].append({"average":self.node_data[node][res]["average"]})
                    # details["nodes"][node].append({"min":self.node_data[node][res]["min"]})

                #print "groundtruthes:", self.groundtruth, self.groundtruth_epsilon, "\n average:", self.node_data[node][res]["average"]
                if self.groundtruth != None and self.groundtruth_epsilon != None:
                    for node in self.node_data:

                        #print "average sum:check", average_sum
                        if math.fabs(self.groundtruth - average_sum) <= self.groundtruth_epsilon:
                            groundtruth_result = True
                        else:
                            groundtruth_result = False

            print "resources mem data: ", average_sum, "\n groundthruth result", groundtruth_result, "details:", details, " \n .................................."
            return "resources_mem", round(average_sum, 3), groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
