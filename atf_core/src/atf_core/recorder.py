#!/usr/bin/env python
import rospy
import rospkg
import rostopic
import rosbag
import rosparam
import yaml
import shutil
import os
import atf_recorder_plugins
import atf_core

from threading import Lock
from atf_msgs.msg import TestblockTrigger, TestStatus


class ATFRecorder:
    def __init__(self, config):
        self.ns = "/aft/"
        self.config = config
        #print "recorder_core: self.config=", self.config

        #self.number_of_tests = rosparam.get_param("/number_of_tests")
        #self.robot_config_file = self.load_data(rosparam.get_param("/robot_config"))

        #if not os.path.exists(rosparam.get_param(rospy.get_name() + "/bagfile_output")):
        #    os.makedirs(rosparam.get_param(rospy.get_name() + "/bagfile_output"))

        #self.topic = "/atf/"
        
        
        #self.test_config = self.load_data(rosparam.get_param(rospy.get_name() + "/test_config_file")
        #                                  )[rosparam.get_param("/test_config")]
        self.test_config = self.config["test_config"][self.config["test_config_name"]]
        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder_plugins") +
                                         "/config/recorder_plugins.yaml")


        # delete test_results directories and create new ones
        if os.path.exists(self.config["bagfile_output"]):
        #    shutil.rmtree(self.config["bagfile_output"])
            pass
        else:
            os.makedirs(self.config["bagfile_output"])

        # create bag file writer handle
        self.lock_write = Lock()
        self.bag = rosbag.Bag(config["bagfile_output"] + self.config["test_name"] + ".bag", 'w')
        self.bag_file_writer = atf_core.BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        self.recorder_plugin_list = []
        if len(recorder_config) > 0:
            for (key, value) in recorder_config.iteritems():
                #print "key=", key
                #print "value=", value
                self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.lock_write,
                                                                                      self.bag_file_writer))

        self.topic_pipeline = []
        self.active_sections = []
        self.requested_topics = []
        self.testblock_list = self.create_testblock_list()

        # Wait for obstacle_distance node
        #rospy.loginfo(rospy.get_name() + ": Waiting for obstacle distance node...")
        #ob_sub = rospy.Subscriber("/atf/obstacle_distance", ObstacleDistance, self.global_topic_callback, queue_size=1,
        #                          callback_args="/atf/obstacle_distance")

        #num_subscriber = ob_sub.get_num_connections()
        #while num_subscriber == 0:
        #    num_subscriber = ob_sub.get_num_connections()

        self.subscriber = []
        self.topics = self.get_topics()
        rospy.Timer(rospy.Duration(0.5), self.create_subscriber_callback)
        rospy.sleep(1) #wait for subscribers to get active (rospy bug?)

        # test status monitoring
        #self.test_status_publisher = rospy.Publisher(self.ns + "test_status", TestStatus, queue_size=10)
        ## Wait for subscriber
        #num_subscriber = self.test_status_publisher.get_num_connections()
        #while num_subscriber == 0:
        #    num_subscriber = self.test_status_publisher.get_num_connections()

        #test_status = TestStatus()
        #test_status.test_name = self.bag_name
        #test_status.status_recording = 1
        #test_status.status_analysing = 0
        #test_status.total = self.number_of_tests

        #self.test_status_publisher.publish(test_status)

        #rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("ATF recorder: started!")

    def shutdown(self):
        rospy.loginfo("Shutdown ATF recorder and close bag file.")
        rospy.sleep(1) # let all threads finish writing to bag file
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

        #test_status = TestStatus()
        #test_status.test_name = self.bag_name
        #test_status.status_recording = 3
        #test_status.status_analysing = 0
        #test_status.total = self.number_of_tests

        #self.test_status_publisher.publish(test_status)

    def create_testblock_list(self):
        #print "creeeeeeeeeeate testblock list"
        testblock_list = {}
        #print "test_config=", self.test_config
        #print "-------------------"
        for testblock in self.test_config:
            #print "testbock=", testblock
            for metric in self.test_config[testblock].keys():
                #print "metric=", metric
                #print "robot_config=", self.config["robot_config"]
                if metric in self.config["robot_config"]:
                    #print "metric is in robot_config"
                    try:
                        testblock_list[testblock]
                    except KeyError:
                        testblock_list[testblock] = self.config["robot_config"][metric]["topics"]
                    else:
                        for topic in self.robot_config_file[metric]["topics"]:
                            #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
                            if topic[0] != "/":
                                topic = "/" + topic
                            testblock_list[testblock].append(topic)
                else:
                    #print "metric is NOT in robot_config"
                    try:
                        #print "self.test_config[testblock][metric]=", self.test_config[testblock][metric]
                        for item in self.test_config[testblock][metric]:
                            #print "item=", item
                            if "topic" in item:
                                if testblock not in testblock_list:
                                    testblock_list.update({testblock: []})
                                topic = item['topic']
                                #print "topic=", topic
                                #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
                                if topic[0] != "/":
                                    topic = "/" + topic
                                testblock_list[testblock].append(topic)
                    except TypeError:
                        pass
        return testblock_list

    def add_requested_topics(self, testblock_name):
        for topic in self.testblock_list[testblock_name]:
            self.requested_topics.append(topic)
            #print "topic=", topic
            if topic not in self.topic_pipeline:
                self.topic_pipeline.append(topic)

    def remove_requested_topics(self, testblock_name):
        for topic in self.testblock_list[testblock_name]:
            self.requested_topics.remove(topic)
            if topic not in self.requested_topics:
                self.topic_pipeline.remove(topic)

    def create_subscriber_callback(self, event):
        #print "self.topics=", self.topics
        for topic in self.topics:
            if topic not in self.subscriber:
                try:
                    msg_class, _, _ = rostopic.get_topic_class(topic)
                    rospy.Subscriber(topic, msg_class, self.global_topic_callback, callback_args=topic)
                    self.subscriber.append(topic)
                except Exception as e:
                    #print e
                    pass

    def record_trigger(self, trigger):
        print "record_trigger: name=", trigger.name, "trigger=", trigger.trigger, "stamp=", trigger.header.stamp

        if trigger.name not in self.test_config:
            raise RecorderError("Testblock '%s' not in test config" % trigger.name)

        # Send message to all recorder plugins
        #print "self.recorder_plugin_list=", self.recorder_plugin_list
        for recorder_plugin in self.recorder_plugin_list:
            recorder_plugin.trigger_callback(trigger)

        # Only process message if testblock requests topics
        print "self.testblock_list=", self.testblock_list
        if trigger.name in self.testblock_list:
            if trigger.trigger == TestblockTrigger.START:
                self.active_sections.append(trigger.name)
                self.add_requested_topics(trigger.name)
            elif trigger.trigger == TestblockTrigger.STOP or trigger.trigger == TestblockTrigger.PAUSE:
                self.active_sections.remove(trigger.name)
                self.remove_requested_topics(trigger.name)
#            elif trigger.trigger == TestblockTrigger.ERROR:
#                self.topic_pipeline = []
            else:
                rospy.loginfo("!!!!!!!!!!!!")

        self.bag_file_writer.write_to_bagfile(self.ns + trigger.name + "/trigger", trigger,
                                  trigger.header.stamp)


    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        if doc == None:
            doc = {}
        return doc

    def global_topic_callback(self, msg, name):
        if name in self.topic_pipeline:
            self.bag_file_writer.write_to_bagfile(name, msg, rospy.Time.now())

    def get_topics(self):
        topics = []
        #print "self.testblock_list=", self.testblock_list
        for testblock in self.testblock_list:
            for topic in self.testblock_list[testblock]:
                if topic not in topics:
                    topics.append(topic)
        return topics

class RecorderError(Exception):
    pass
