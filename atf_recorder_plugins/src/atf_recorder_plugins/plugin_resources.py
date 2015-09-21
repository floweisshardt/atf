#!/usr/bin/env python
import rospy
import psutil
import time
import xmlrpclib
import rosnode

from copy import copy
from re import findall
from subprocess import check_output, CalledProcessError
from atf_msgs.msg import *
from atf_recorder import BagfileWriter


class RecordResources:
    def __init__(self, topic_prefix, config_file, write_lock, bag_file):
        self.topic_prefix = topic_prefix
        self.test_config = config_file

        self.resources_timer_frequency = 4.0  # Hz
        self.timer_interval = 1/self.resources_timer_frequency

        self.testblock_list = self.create_testblock_list()
        self.pid_list = self.create_pid_list()
        self.requested_nodes = {}
        self.res_pipeline = {}

        self.BfW = BagfileWriter(bag_file, write_lock)

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)

    def update_requested_nodes(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE:
            for node in self.testblock_list[msg.name]:
                if node not in self.requested_nodes:
                    self.requested_nodes[node] = copy(self.testblock_list[msg.name][node])
                    self.res_pipeline[node] = copy(self.testblock_list[msg.name][node])
                else:
                    for res in self.testblock_list[msg.name][node]:
                        self.requested_nodes[node].append(res)
                        if res not in self.res_pipeline[node]:
                            self.res_pipeline[node].append(res)

        elif msg.trigger.trigger == Trigger.FINISH:
            for node in self.testblock_list[msg.name]:
                for res in self.testblock_list[msg.name][node]:
                    self.requested_nodes[node].remove(res)
                    if res not in self.requested_nodes[node]:
                        self.res_pipeline[node].remove(res)
                    if len(self.requested_nodes[node]) == 0:
                        del self.requested_nodes[node]
                        del self.res_pipeline[node]

    def create_testblock_list(self):

        testblock_list = {}
        for testblock in self.test_config:
            try:
                self.test_config[testblock]["resources"]
            except KeyError:
                continue
            else:
                for resource in self.test_config[testblock]["resources"]:
                    try:
                        testblock_list[testblock]
                    except KeyError:
                        testblock_list.update({testblock: {}})

                    for node_name in self.test_config[testblock]["resources"][resource]:

                        if node_name not in testblock_list[testblock]:
                            testblock_list[testblock].update({node_name: [resource]})
                        elif resource not in testblock_list[testblock][node_name]:
                            testblock_list[testblock][node_name].append(resource)

        return testblock_list

    def collect_resource_data(self, event):
        pipeline = copy(self.res_pipeline)
        if not len(pipeline) == 0:
            msg = Resources()
            topic = self.topic_prefix + "resources"

            for node in pipeline:
                msg_data = NodeResources()
                pid = self.pid_list[node]

                if pid is None:
                    continue

                try:
                    msg_data.node_name = node

                    if "cpu" in pipeline[node]:
                        msg_data.cpu = psutil.Process(pid).cpu_percent(interval=self.timer_interval)

                    if "mem" in pipeline[node]:
                        msg_data.memory = psutil.Process(pid).memory_percent()

                    if "io" in pipeline[node]:
                        data = findall('\d+', str(psutil.Process(pid).io_counters()))
                        msg_data.io.read_count = int(data[0])
                        msg_data.io.write_count = int(data[1])
                        msg_data.io.read_bytes = int(data[2])
                        msg_data.io.write_bytes = int(data[3])

                    if "network" in pipeline[node]:
                        data = findall('\d+', str(psutil.net_io_counters()))
                        msg_data.network.bytes_sent = int(data[0])
                        msg_data.network.bytes_recv = int(data[1])
                        msg_data.network.packets_sent = int(data[2])
                        msg_data.network.packets_recv = int(data[3])
                        msg_data.network.errin = int(data[4])
                        msg_data.network.errout = int(data[5])
                        msg_data.network.dropin = int(data[6])
                        msg_data.network.dropout = int(data[7])

                    msg.nodes.append(msg_data)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            self.BfW.write_to_bagfile(topic, msg, rospy.Time.from_sec(time.time()))

    def trigger_callback(self, msg):

        # Only save node resources if testblock requests them
        if msg.name in self.testblock_list:
            self.update_requested_nodes(msg)

        if msg.trigger.trigger == Trigger.ERROR:
            self.res_pipeline = []

    def create_pid_list(self):
        node_list = {}
        for (testblock, nodes) in self.testblock_list.iteritems():
            for node in nodes:
                if node not in node_list:
                    node_list[node] = self.get_pid(node)

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
