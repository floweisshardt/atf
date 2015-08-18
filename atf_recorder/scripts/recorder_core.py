#!/usr/bin/env python
import rospy
import rospkg
import rostopic
import rosbag
import rosparam
import yaml
import time
import os

from threading import Lock
from atf_msgs.msg import *
from atf_recorder.srv import *
import atf_recorder


class ATFRecorder:
    def __init__(self):

        bag_name = rosparam.get_param("/test_name")
        self.robot_config_file = self.load_data(rosparam.get_param(rospy.get_name() + "/robot_config_file"))

        if not os.path.exists(rosparam.get_param("/recorder/bagfile_output")):
            os.makedirs(rosparam.get_param("/recorder/bagfile_output"))

        self.topic = "/testing/"
        self.lock_write = Lock()
        self.bag = rosbag.Bag(rosparam.get_param("/recorder/bagfile_output") + bag_name + ".bag", 'w')
        self.test_config = self.load_data(rosparam.get_param("/recorder/test_config_file"))[rosparam.get_param(
            "/test_config")]
        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder") + "/config/recorder_config.yaml")

        # Init metric recorder
        self.recorder_list = []
        for item in recorder_config:
            self.recorder_list.append(getattr(atf_recorder, recorder_config[item]["class"])(self.topic,
                                                                                            self.test_config,
                                                                                            self.lock_write,
                                                                                            self.bag))

        self.topic_pipeline = []
        self.active_sections = []
        self.requested_topics = []
        self.testblock_list = self.create_testblock_list()

        for topic in self.get_topics():
            msg_type = rostopic.get_topic_class(topic, blocking=True)[0]
            rospy.Subscriber(topic, msg_type, self.global_topic_callback, queue_size=5, callback_args=topic)

        rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def create_testblock_list(self):
        testblock_list = {}
        for testblock in self.test_config:
            for metric in self.test_config[testblock]:

                # Topics
                if metric in self.robot_config_file:
                    try:
                        testblock_list[testblock]["topics"]
                    except KeyError:
                        testblock_list.update({testblock: {"topics": self.robot_config_file[testblock]["topics"]}})
                    else:
                        for topic in self.robot_config_file[testblock]["topics"]:
                            testblock_list[testblock]["topics"].append(topic)

        return testblock_list

    def update_requested_topics(self, name, command):

        if command == "add":
            for topic in self.testblock_list[name]["topics"]:
                self.requested_topics.append(topic)
                if topic not in self.topic_pipeline:
                    self.topic_pipeline.append(topic)

        elif command == "del":
            for topic in self.testblock_list[name]["topics"]:
                self.requested_topics.remove(topic)
                if topic not in self.requested_topics:
                    self.topic_pipeline.remove(topic)

    def command_callback(self, msg):

        if (msg.trigger.trigger == Trigger.ACTIVATE and msg.name in self.active_sections) or\
                (msg.trigger.trigger == Trigger.FINISH and msg.name not in self.active_sections) or\
                msg.name not in self.test_config:
            return RecorderCommandResponse(False)

        for recorder in self.recorder_list:
            recorder.trigger_callback(msg)

        if msg.trigger.trigger == Trigger.ACTIVATE:
            self.update_requested_topics(msg.name, "add")
            self.active_sections.append(msg.name)
            # rospy.loginfo("Section '" + msg.name + "': ACTIVATE")
        elif msg.trigger.trigger == Trigger.FINISH:
            self.update_requested_topics(msg.name, "del")
            self.active_sections.remove(msg.name)
            # rospy.loginfo("Section '" + msg.name + "': FINISH")
        elif msg.trigger.trigger == Trigger.ERROR:
            self.topic_pipeline = []
            # rospy.loginfo("Section '" + msg.name + "': ERROR")

        self.write_to_bagfile(self.topic + msg.name + "/Trigger", Trigger(msg.trigger.trigger),
                              rospy.Time.from_sec(time.time()))

        return RecorderCommandResponse(True)

    @staticmethod
    def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc

    def global_topic_callback(self, msg, name):
        if name in self.topic_pipeline:
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

        for item in self.test_config:
            for metric in self.test_config[item]:
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
