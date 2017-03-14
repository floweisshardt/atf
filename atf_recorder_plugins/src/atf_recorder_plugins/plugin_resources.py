#!/usr/bin/env python
import rospy
import psutil
import time
import xmlrpclib
import rosnode
import yaml

from copy import copy
from re import findall
from subprocess import check_output, CalledProcessError
from atf_msgs.msg import NodeResources, Resources, IO, Network, TestblockTrigger

class RecordResources:
    def __init__(self, write_lock, bag_file_writer):
        self.topic_prefix = "atf/"
        file = "/home/fmw-hb/workshop_catkin_ws/src/msh_test/hannes_test_msh/config/test_configs/test2.yaml"
        with open(file, 'r') as stream:
            self.test_config = yaml.load(stream)

        self.resources_timer_frequency = 10.0  # Hz
        self.timer_interval = 1/self.resources_timer_frequency

        self.testblock_list = self.create_testblock_list()
        self.pid_list = self.create_pid_list()
        self.requested_nodes = {}
        self.res_pipeline = {}

        self.BfW = bag_file_writer

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)

    def update_requested_nodes(self, msg):
        counter = 0
        if msg.trigger == TestblockTrigger.START:
            print "START Trigger"
            for node in self.testblock_list[msg.name]:

                for resource, names in node.iteritems():
                    for name in names:
                        print "resource:", resource," name", name
                        if name not in self.requested_nodes:
                            print "new node", counter, self.testblock_list[msg.name][counter][resource]#, "ohne: ", self.testblock_list[msg.name][resource]
                            self.requested_nodes[resource] = copy(self.testblock_list[msg.name][counter][resource])
                            self.res_pipeline[resource] = copy(self.testblock_list[msg.name][counter][resource])
                counter += 1

        elif msg.trigger == TestblockTrigger.STOP:
            print "STOP Trigger"

    def create_testblock_list(self):

        testblock_list = {}
        nodes = []
        for testblock in self.test_config:
            #print "\n testblock:", testblock
            try:
                self.test_config[testblock]["resources"]
            except KeyError:
                continue
            else:
                for resource in self.test_config[testblock]["resources"]:
                    #print "resources:", resource
                    try:
                        testblock_list[testblock]
                    except KeyError:
                        testblock_list.update({testblock: {}})

                    if 'groundtruth' in resource:
                        del resource['groundtruth']
                    if 'groundtruth_epsilon' in resource:
                        del resource['groundtruth_epsilon']

                    for node_name in resource:
                        nodes.append(resource)
                        #print "node", node_name, "appended", nodes

            testblock_list.update({testblock: nodes})
        #print "\n testblock list:", testblock_list
        return testblock_list

    def collect_resource_data(self, event):
        pipeline = copy(self.res_pipeline)
        if not len(pipeline) == 0:
            msg = Resources()
            msg_list = []
            topic = self.topic_prefix + "resources"
            for resource, nodes in pipeline.iteritems():
                msg_data = NodeResources()
                #print "pid list: ", self.pid_list, "pid", self.pid_list[resource]
                for pid in self.pid_list[resource]:

                    if pid is None:
                        continue
                    for node in nodes:
                        #print "message node:", node, "resource:", resource, "\n pipeline:", pipeline
                        try:
                            msg_data.node_name = node

                            if "cpu" in pipeline:
                                msg_data.cpu = psutil.Process(pid).get_cpu_percent(interval=self.timer_interval)

                            if "mem" in pipeline:
                                msg_data.memory = psutil.Process(pid).get_memory_percent()

                            if "io" in pipeline:
                                data = findall('\d+', str(psutil.Process(pid).get_io_counters()))
                                msg_data.io.read_count = int(data[0])
                                msg_data.io.write_count = int(data[1])
                                msg_data.io.read_bytes = int(data[2])
                                msg_data.io.write_bytes = int(data[3])

                            if "network" in pipeline:
                                data = findall('\d+', str(psutil.net_io_counters()))
                                msg_data.network.bytes_sent = int(data[0])
                                msg_data.network.bytes_recv = int(data[1])
                                msg_data.network.packets_sent = int(data[2])
                                msg_data.network.packets_recv = int(data[3])
                                msg_data.network.errin = int(data[4])
                                msg_data.network.errout = int(data[5])
                                msg_data.network.dropin = int(data[6])
                                msg_data.network.dropout = int(data[7])

                            #print "message data: ", msg_data
                            msg_list.append(copy(msg_data))
                            #print "message node name: \n", msg_data.node_name, "\n type:", type(msg_data.node_name)
                        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                            rospy.logerr("collecting error: %s", e)
                            pass
            msg.nodes=msg_list

            self.BfW.write_to_bagfile(topic, msg, rospy.Time.now())

    def trigger_callback(self, msg):

        # Only save node resources if testblock requests them
        #print "trigger callback: msg \n", msg, " \n testblocks", self.testblock_list, "\n msg trigger:", msg.trigger
        if msg.name in self.testblock_list:
            self.update_requested_nodes(msg)


    def create_pid_list(self):
        node_list = {}
        pid_list = []
        for (testblock, nodes) in self.testblock_list.iteritems():
            for node in nodes:
                for resource, names in node.iteritems():
                    print "node: ", node,"resource", resource,"nodes: ", nodes, "node_list:", node_list, "\n names:", names
                    if isinstance(names, list):
                        for name in names:
                            if self.get_pid(name) not in pid_list:
                                pid_list.append(self.get_pid(name))
                        node_list.update({resource:pid_list})
        #print "node list", node_list
        return node_list

    @staticmethod
    def get_pid(name):
        try:
            pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
            return pid[0]
        except IndexError:
            pass

        try:
            node_id = '/NODEINFO'
            node_api = rosnode.get_api_uri(rospy.get_master(), name)
            code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid(node_id)
        except IOError:
            pass
        else:
            return pid

        try:
            return int(check_output(["pidof", "-s", name]))
        except CalledProcessError:
            pass

        rospy.logerr("Node '" + name + "' is not running!")
        return None