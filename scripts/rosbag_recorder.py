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
        self.lock_write = Lock()
        self.lock_pipeline = Lock()
        self.timer_interval = 0.01
        self.tf_active = False

        self.bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/" +
                              rosparam.get_param("/test_name") + ".bag", 'w')

        self.path = rospkg.RosPack().get_path("cob_benchmarking") + "/tests/test_config.yaml"
        self.data = self.load_data(self.path)[rosparam.get_param("/test_name")]

        self.nodes = {}
        self.pipeline = {}
        self.requested_nodes = {"cpu": [], "mem": [], "io": [], "network": [], "path_length": []}

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_resource_data)
        msg_type = rostopic.get_topic_class("/tf", blocking=True)[0]

        rospy.Subscriber("/tf", msg_type, self.tf_callback, queue_size=1)
        self.sub_recorder_commands = rospy.Subscriber(self.topic + "recorder_commands", Recorder, self.manage_recording,
                                                      queue_size=1)

        while not rospy.is_shutdown() and self.sub_recorder_commands.get_num_connections() == 0:
            rospy.sleep(0.01)

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

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bag.close()

    def manage_recording(self, msg):

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.update_requested_nodes(msg.name, "add")
            rospy.loginfo("Section '" + msg.name + "': ACTIVATE")
        elif msg.trigger.trigger == Trigger.FINISH:
            self.update_requested_nodes(msg.name, "del")
            rospy.loginfo("Section '" + msg.name + "': FINISH")

        msg_trigger = Trigger()
        msg_time = Time()

        msg_trigger.trigger = msg.trigger.trigger
        msg_time.timestamp = msg.timestamp

        self.write_to_bagfile([[self.topic + msg.name + "/Trigger", msg_trigger],
                               [self.topic + msg.name + "/Time", msg_time]], rospy.Time.now())

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
                try:
                    pid = self.nodes[node]
                except KeyError:
                    continue

                try:
                    msg_data.node_name = str(psutil.Process(pid).name().split(".")[0])

                    if "cpu" in self.pipeline[node]:
                        msg_data.cpu = psutil.Process(pid).cpu_percent(interval=self.timer_interval)

                    if "mem" in self.pipeline[node]:
                        msg_data.memory = psutil.Process(pid).memory_percent()

                    if "io" in self.pipeline[node]:
                        data = findall('\d+', str(psutil.Process(pid).io_counters()))
                        msg_data.io.read_count = int(data[0])
                        msg_data.io.write_count = int(data[1])
                        msg_data.io.read_bytes = int(data[2])
                        msg_data.io.write_bytes = int(data[3])

                    if "network" in self.pipeline[node]:
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

            self.write_to_bagfile([[topic, msg]], event.current_real)

        self.lock_pipeline.release()

    @staticmethod
    def get_pid(name):
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]

    def tf_callback(self, msg):
        if self.tf_active:
            self.write_to_bagfile([[self.topic + "tf", msg]], rospy.Time.now())

    def write_to_bagfile(self, data, time):
        self.lock_write.acquire()
        for value in data:
            self.bag.write(value[0], value[1], time)
        self.lock_write.release()


if __name__ == "__main__":
    rospy.init_node('rosbag_recorder')
    with RosBagRecorder() as recorder:
        while not recorder.sub_recorder_commands.get_num_connections() == 0 or rospy.is_shutdown():
            rospy.sleep(0.01)
