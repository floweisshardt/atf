#!/usr/bin/env python
import rospy
import rostopic
import psutil
import rosbag
import rosparam
import yaml
import time
import os
import xmlrpclib
import rosnode
from subprocess import check_output, CalledProcessError

from re import findall
from threading import Lock
from atf_msgs.msg import *
from atf_recorder.srv import *


class ATFRecorder:
    def __init__(self):

        self.topic = "/testing/"
        self.lock_write = Lock()
        self.resources_timer_frequency = 4.0  # Hz
        self.timer_interval = 1/self.resources_timer_frequency
        self.tf_active = False

        bag_name = rosparam.get_param("/test_name")

        if not os.path.exists(rosparam.get_param("/recorder/bagfile_output")):
            os.makedirs(rosparam.get_param("/recorder/bagfile_output"))

        self.bag = rosbag.Bag(rosparam.get_param("/recorder/bagfile_output") + bag_name + ".bag", 'w')

        self.robot_config_file = self.load_data(rosparam.get_param(rospy.get_name() + "/robot_config_file"))
        self.config_data = self.load_data(rosparam.get_param("/recorder/test_config_file"))[rosparam.get_param(
            "/test_config")]

        # TODO: Pipeline for topics. Same style as res_pipeline
        topics = self.get_topics()

        self.nodes = {}
        self.res_pipeline = {}
        self.topic_pipeline = {}
        self.active_sections = []

        # TODO: No hardcoded metrics
        self.requested_nodes = {"cpu": [], "mem": [], "io": [], "network": [], "path_length": []}

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)

        for topic in topics:
            msg_type = rostopic.get_topic_class(topic, blocking=True)[0]
            rospy.Subscriber(topic, msg_type, self.global_topic_callback, queue_size=5, callback_args=topic)

        rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)

        # ---- GET PIDS FOR THE NODES ----
        for section in self.config_data:
            try:
                resources = self.config_data[section]["resources"]
            except KeyError:
                pass
            else:
                for res in resources:
                    if not type(resources[res]) == bool:
                        for item in resources[res]:
                            if item not in self.nodes:
                                try:
                                    self.nodes[item] = self.get_pid(item)
                                except False:
                                    rospy.logerr("Node '" + item + "' is not running or does not exist!")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def command_callback(self, msg):

        if msg.name in self.config_data:

            if (msg.trigger.trigger == Trigger.ACTIVATE and msg.name in self.active_sections) or\
                    (msg.trigger.trigger == Trigger.FINISH and msg.name not in self.active_sections):
                return RecorderCommandResponse(True)

            if msg.trigger.trigger == Trigger.ACTIVATE:
                self.update_requested_nodes(msg.name, "add")
                self.active_sections.append(msg.name)
                # rospy.loginfo("Section '" + msg.name + "': ACTIVATE")
            elif msg.trigger.trigger == Trigger.FINISH:
                self.update_requested_nodes(msg.name, "del")
                self.active_sections.remove(msg.name)
                # rospy.loginfo("Section '" + msg.name + "': FINISH")
            elif msg.trigger.trigger == Trigger.ERROR:
                self.res_pipeline = {}
                self.tf_active = False
                # rospy.loginfo("Section '" + msg.name + "': ERROR")

            self.write_to_bagfile(self.topic + msg.name + "/Trigger", Trigger(msg.trigger.trigger),
                                  rospy.Time.from_sec(time.time()))

        return RecorderCommandResponse(True)

    def update_requested_nodes(self, name, command):
        update_pipeline = False

        try:
            self.config_data[name]["path_length"]
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
            resources_temp = self.config_data[name]["resources"]
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

    @staticmethod
    def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc

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

        return False

    def global_topic_callback(self, msg, name):
        if self.tf_active:
            now = rospy.Time.from_sec(time.time())
            for item in msg.transforms:
                item.header.stamp = now
            self.write_to_bagfile(name, msg, now)

    def write_to_bagfile(self, topic, data, set_time):
        self.lock_write.acquire()
        self.bag.write(topic, data, set_time)
        self.lock_write.release()

    def get_topics(self):
        topics = []

        for item in self.config_data:
            for metric in self.config_data[item]:
                if metric in self.robot_config_file:
                    for topic in self.robot_config_file[metric]["topics"]:
                        if topic not in topics:
                            topics.append(topic)

        return topics


if __name__ == "__main__":
    rospy.init_node('atf_recorder')
    with ATFRecorder():
        while not rospy.is_shutdown():
            pass
