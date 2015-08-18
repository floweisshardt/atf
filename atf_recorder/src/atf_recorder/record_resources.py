#!/usr/bin/env python
import rospy
import psutil
from re import findall
import time
import xmlrpclib
import rosnode

from subprocess import check_output, CalledProcessError
from atf_msgs.msg import *


class RecordResources:
    def __init__(self, topic, config_file, lock, bag_file):
        self.topic = topic
        self.test_config = config_file
        self.lock = lock
        self.bag_file = bag_file

        self.resources_timer_frequency = 4.0  # Hz
        self.timer_interval = 1/self.resources_timer_frequency

        self.testblock_list = self.create_testblock_list()
        self.requested_nodes = {}
        self.res_pipeline = []

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)

    def update_requested_nodes(self, name, command):
        update_pipeline = False

        try:
            self.test_config[name]["path_length"]
        except KeyError:
            pass
        else:
            if command == "add":
                self.tf_active = True
                self.requested_nodes["path_length"].append(True)
            elif command == "del":
                self.tf_active = False
                self.requested_nodes["path_length"].remove(True)
            update_pipeline = True

        try:
            resources_temp = self.test_config[name]["resources"]
        except KeyError:
            pass
        else:
            for res in resources_temp:
                for item in resources_temp[res]:
                    if command == "add":
                        self.requested_nodes[res].append(item)
                    elif command == "del":
                        self.requested_nodes[res].remove(item)
            update_pipeline = True

        if update_pipeline:
            self.res_pipeline = {}

            for item in self.requested_nodes:
                for value in self.requested_nodes[item]:
                    if not type(value) == bool:
                        if value not in self.res_pipeline:
                            self.res_pipeline[value] = [item]
                        elif item not in self.res_pipeline[value]:
                            self.res_pipeline[value].append(item)
                    else:
                        self.tf_active = value

    def create_testblock_list(self):
        testblock_list = {}
        for testblock in self.test_config:
            for resource in self.test_config[testblock]["resources"]:
                try:
                    testblock_list[testblock]
                except KeyError:
                    testblock_list.update({testblock: {}})

                for node_name in self.test_config[testblock]["resources"][resource]:

                    if node_name not in testblock_list[testblock]:
                        testblock_list[testblock].update({node_name: {"pid": self.get_pid(node_name),
                                                                      "resources": [resource]}})
                    elif resource not in testblock_list[testblock][node_name]["resources"]:
                        testblock_list[testblock][node_name]["resources"].append(resource)

        return testblock_list

    def collect_resource_data(self, event):
        pipeline = self.res_pipeline
        if not len(pipeline) == 0:
            msg = Resources()
            topic = self.topic + "Resources"

            for node in pipeline:
                msg_data = NodeResources()
                try:
                    pid = self.nodes[node]
                except KeyError:
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

            self.write_to_bagfile(topic, msg, rospy.Time.from_sec(time.time()))

    def trigger_callback(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.update_requested_nodes(msg.name, "add")
        elif msg.trigger.trigger == Trigger.FINISH:
            self.update_requested_nodes(msg.name, "del")
        elif msg.trigger.trigger == Trigger.ERROR:
            self.res_pipeline = []

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

        rospy.logerr("Node '" + name + "' is not running or does not exist!")
        return None

    def write_to_bagfile(self, topic, data, set_time):
        self.lock.acquire()
        self.bag_file.write(topic, data, set_time)
        self.lock.release()
