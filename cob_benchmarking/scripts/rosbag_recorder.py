#!/usr/bin/env python
import rospy
import rostopic
import psutil
import xmlrpclib
import rosbag
import rospkg
import rosparam
import yaml
import time

from re import findall
from threading import Lock
from atf_msgs.msg import *
from cob_benchmarking.srv import *


class RosBagRecorder:
    def __init__(self):

        self.topic = "/testing/"
        self.lock_write = Lock()
        self.resources_timer_frequency = 5.0  # Hz
        self.timer_interval = 1/self.resources_timer_frequency
        self.tf_active = False

        self.bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/" +
                              rosparam.get_param("/test_name") + ".bag", 'w')

        test_config_path = rospkg.RosPack().get_path("cob_benchmarking") + "/config/test_config.yaml"
        robot_config_path = rospkg.RosPack().get_path("cob_benchmarking") + "/config/robot_config.yaml"
        tf_topic = self.load_data(robot_config_path)["topics"]["path"]["tf_topic"][0]
        self.config_data = self.load_data(test_config_path)[rosparam.get_param("/test_name")]

        self.nodes = {}
        self.pipeline = {}
        self.active_sections = []
        self.requested_nodes = {"cpu": [], "mem": [], "io": [], "network": [], "path_length": []}

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)
        msg_type = rostopic.get_topic_class("/tf", blocking=True)[0]
        rospy.Subscriber(tf_topic, msg_type, self.tf_callback, queue_size=1)
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
                                except IndexError:
                                    pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def command_callback(self, msg):

        if (msg.trigger.trigger == Trigger.ACTIVATE and msg.name in self.active_sections) or\
                (msg.trigger.trigger == Trigger.FINISH and msg.name not in self.active_sections):
            return RecorderCommandResponse(True)

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.update_requested_nodes(msg.name, "add")
            self.active_sections.append(msg.name)
            rospy.loginfo("Section '" + msg.name + "': ACTIVATE")
        elif msg.trigger.trigger == Trigger.FINISH:
            self.update_requested_nodes(msg.name, "del")
            self.active_sections.remove(msg.name)
            rospy.loginfo("Section '" + msg.name + "': FINISH")
        elif msg.trigger.trigger == Trigger.ERROR:
            self.pipeline = {}
            self.tf_active = False
            rospy.loginfo("Section '" + msg.name + "': ERROR")

        self.write_to_bagfile(self.topic + msg.name + "/Trigger", Trigger(msg.trigger.trigger),
                              rospy.Time.from_sec(time.time()))

        return RecorderCommandResponse(True)

    def update_requested_nodes(self, name, command):
        update_pipeline = False

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

        try:
            self.config_data[name]["path_length"]
        except KeyError:
            pass
        else:
            if command == "add":
                self.requested_nodes["path_length"].append(True)
            elif command == "del":
                self.requested_nodes["path_length"].remove(True)
            update_pipeline = True

        if update_pipeline:
            self.pipeline = {}
            self.tf_active = False

            for item in self.requested_nodes:
                for value in self.requested_nodes[item]:
                    if not type(value) == bool:
                        if value not in self.pipeline:
                            self.pipeline[value] = [item]
                        elif item not in self.pipeline[value]:
                            self.pipeline[value].append(item)
                    elif value:
                            self.tf_active = value

    @staticmethod
    def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc

    def collect_resource_data(self, event):
        pipeline = self.pipeline
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
                    msg_data.node_name = str(psutil.Process(pid).name().split(".")[0])

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
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]

    def tf_callback(self, msg):
        if self.tf_active:
            now = rospy.Time.from_sec(time.time())
            for item in msg.transforms:
                item.header.stamp = now
            self.write_to_bagfile(self.topic + "tf", msg, now)

    def write_to_bagfile(self, topic, data, set_time):
        self.lock_write.acquire()
        self.bag.write(topic, data, set_time)
        self.lock_write.release()


if __name__ == "__main__":
    rospy.init_node('rosbag_recorder')
    with RosBagRecorder():
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
