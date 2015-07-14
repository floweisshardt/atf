#!/usr/bin/env python
import rospy
import rostopic
import psutil
import rosbag
import rospkg
import rosparam
import yaml

from re import findall
from threading import Lock
from atf_msgs.msg import *
from cob_benchmarking.msg import Recorder


class RosBagRecorder:
    def __init__(self):

        self.topic = "/testing/"
        self.node_names = rospy.get_param("/node_names")
        self.lock_write = Lock()
        self.lock_pipeline = Lock()
        self.timer_interval = 0.05
        self.tf_active = False

        self.bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/" +
                              rosparam.get_param("/test_name") + "_data.bag", 'w')

        self.path = rospkg.RosPack().get_path("cob_benchmarking") + "/tests/test_config.yaml"
        self.data = self.load_data(self.path)[rosparam.get_param("/test_name")]

        self.nodes = {}
        self.pipeline = {}
        self.requested_nodes = {"cpu": [], "mem": [], "io": [], "network": [], "path_length": []}

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)
        msg_type = rostopic.get_topic_class("/tf", blocking=True)[0]

        rospy.Subscriber("/tf", msg_type, self.tf_callback, queue_size=1)
        sub_recorder_commands = rospy.Subscriber(self.topic + "recorder_commands", Recorder, self.manage_recording,
                                                 queue_size=1)

        while not rospy.is_shutdown() and sub_recorder_commands.get_num_connections() == 0:
            rospy.sleep(0.05)

        # ---- GET PIDS FOR THE NODES ----
        for section in self.data:
            try:
                resources = self.data[section]["resources"]
            except KeyError:
                pass
            else:
                for res in resources:
                    if not type(resources[res]) == bool:
                        for item in resources[res]:
                            if item not in self.nodes:
                                try:
                                    self.nodes.update({item: self.get_pid(item)})
                                except IndexError:
                                    pass

    def __del__(self):
        trigger_bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/" +
                                 rosparam.get_param("/test_name") + "_trigger.bag", 'w')

    @staticmethod
    def read_bagfile(path):

        bag = rosbag.Bag(path)
        planning = [msg for (topic, msg, t) in bag.read_messages(topics=['planning_timer'])]
        execution = [msg for (topic, msg, t) in bag.read_messages(topics=['execution_timer'])]
        scene_infos = [msg for (topic, msg, t) in bag.read_messages(topics=['scene_informations'])]
        ressource_info = [msg for (topic, msg, t) in bag.read_messages(topics=['ressource_data'])]
        planning_error = [msg for (topic, msg, t) in bag.read_messages(topics=['planning_error'])]
        tf_data = [msg for (topic, msg, t) in bag.read_messages(topics=['tf'])]
        bag.close()

    def manage_recording(self, msg):
        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.update_requested_nodes(msg.name, "add")

        elif msg.trigger.trigger == Trigger.FINISH:
            self.update_requested_nodes(msg.name, "del")

    def update_requested_nodes(self, name, command):
        try:
            resources_temp = self.data[name]["resources"]
        except KeyError:
            pass
        else:
            for res in resources_temp:
                if not type(resources_temp[res]) == bool:
                    for item in resources_temp[res]:
                        if command == "add":
                            self.requested_nodes[res].append(item)
                        elif command == "del":
                            self.requested_nodes[res].remove(item)

        try:
            self.data[name]["path_length"]
        except KeyError:
            pass
        else:
            if command == "add":
                self.requested_nodes["path_length"].append(True)
            elif command == "del":
                self.requested_nodes["path_length"].remove(True)

        self.lock_pipeline.acquire()
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

        self.lock_pipeline.release()

    @staticmethod
    def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc

    def collect_resource_data(self, event):
        self.lock_pipeline.acquire()
        if not len(self.pipeline) == 0:
            msg = Resources()
            topic = self.topic + "Resources"

            for node in self.pipeline:
                msg_data = NodeResources()
                pid = self.nodes[node]

                try:
                    if "cpu" in self.pipeline[node]:
                        msg_data.cpu = psutil.Process(pid).cpu_percent(interval=self.timer_interval)
                    if "mem" in self.pipeline[node]:
                        msg_data.memory = psutil.Process(pid).memory_percent()
                    if "io" in self.pipeline[node]:
                        data = findall('\d+', str(psutil.Process(pid).io_counters()))
                        msg_data.io.read_count = data[0]
                        msg_data.io.write_count = data[1]
                        msg_data.io.read_bytes = data[2]
                        msg_data.io.write_bytes = data[3]
                    if "network" in self.pipeline[node]:
                        data = findall('\d+', str(psutil.net_io_counters()))
                        msg_data.network.bytes_sent = data[0]
                        msg_data.network.bytes_recv = data[1]
                        msg_data.network.packets_sent = data[2]
                        msg_data.network.packets_recv = data[3]
                        msg_data.network.errin = data[4]
                        msg_data.network.errout = data[5]
                        msg_data.network.dropin = data[6]
                        msg_data.network.dropout = data[7]

                    msg.nodes.append(msg_data)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            self.lock_write.acquire()
            self.bag.write(topic, msg, event.current_real)
            self.lock_write.release()

        self.lock_pipeline.release()

    @staticmethod
    def get_pid(name):
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]

    def tf_callback(self, msg):
        if self.tf_active:
            self.lock_write.acquire()
            self.bag.write(self.topic + "tf", msg, rospy.Time.now())
            self.lock_write.release()


if __name__ == "__main__":
    rospy.init_node('rosbag_recorder')
    RosBagRecorder()
    while not rospy.is_shutdown():
        rospy.spin()
