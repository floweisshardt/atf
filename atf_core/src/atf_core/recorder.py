#!/usr/bin/env python
import rospy
import rospkg
import rostopic
import rosbag
import yaml
import os
import atf_recorder_plugins
import atf_core

from threading import Lock
from atf_msgs.msg import TestblockTrigger


class ATFRecorder:
    def __init__(self, test):
        self.ns = "/atf/"
        self.test = test
        #print "recorder_core: self.test.name:", self.test.name

        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder_plugins") +
                                         "/config/recorder_plugins.yaml")

        try:
            # delete test_results directories and create new ones
            if os.path.exists(self.test.generation_config["bagfile_output"]):
            #    shutil.rmtree(self.tests.generation_config"bagfile_output"]) #FIXME will fail if multiple test run concurrently
                pass
            else:
                os.makedirs(self.test.generation_config["bagfile_output"])
        except OSError:
            pass

        # lock for self variables of this class
        self.lock = Lock()

        # create bag file writer handle
        self.lock_write = Lock()
        self.bag = rosbag.Bag(self.test.generation_config["bagfile_output"] + self.test.name + ".bag", 'w')
        self.bag_file_writer = atf_core.BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        self.recorder_plugin_list = []
        #print "recorder_config", recorder_config
        if len(recorder_config) > 0:
            for value in recorder_config.values():
                #print "value=", value
                self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.lock_write,
                                                                                      self.bag_file_writer))
        #print "self.recorder_plugin_list", self.recorder_plugin_list

        self.active_topics = {}

        #rospy.Service(self.topic + "recorder_command", RecorderCommand, self.command_callback)
        rospy.on_shutdown(self.shutdown)
        
        # wait for topics and services to become active
        for topic in test.robot_config["wait_for_topics"]:
            rospy.loginfo("Waiting for topic '%s'...", topic)
            rospy.wait_for_message(topic, rospy.AnyMsg)
            rospy.loginfo("... got message on topic '%s'.", topic)

        for service in test.robot_config["wait_for_services"]:
            rospy.loginfo("Waiting for service '%s'...", service)
            rospy.wait_for_service(service)
            rospy.loginfo("... service '%s' available.", service)

        rospy.sleep(1) #wait for subscribers to get active (rospy bug?)
        rospy.loginfo("ATF recorder: started!")

    def shutdown(self):
        rospy.loginfo("Shutdown ATF recorder and close bag file.")
        rospy.sleep(1) # let all threads finish writing to bag file
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def create_subscriber(self, topic):
        if not topic.startswith("/"):
            msg = "topic %s is not a global topic. Needs to start with '/'"%topic
            print msg
            raise ATFRecorderError(msg)
        try:
            msg_class, _, _ = rostopic.get_topic_class(topic)
            subscriber = rospy.Subscriber(topic, msg_class, self.global_topic_callback, callback_args=topic)
        except Exception as e:
            msg = "Error while add a subscriber for %s: %s."%(topic, e)
            print msg
            raise ATFRecorderError(msg)
        return subscriber

    def record_status(self, status):
        #print "status", status
        self.bag_file_writer.write_to_bagfile("/atf/status", status, status.stamp)

    def start_recording(self, testblock_name):
        self.lock.acquire()
        if testblock_name not in self.test.test_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % testblock_name)

        for topic in self.get_topics_of_testblock(testblock_name):
            if topic not in self.active_topics.keys():
                # add new entry to active topics
                self.active_topics[topic] = {}
                self.active_topics[topic]["testblocks"] = [testblock_name]
                self.active_topics[topic]["subscriber"] = self.create_subscriber(topic)
            else:
                self.active_topics[topic]["testblocks"].append(testblock_name)

        #print ">>> start recording for %s, active_topics="%testblock_name, self.active_topics
        self.lock.release()

        # Send message to all recorder plugins
        #print "self.recorder_plugin_list=", self.recorder_plugin_list
        for recorder_plugin in self.recorder_plugin_list:
            #FIXME: need to filter the topics not needed for current trigger
            recorder_plugin.trigger_callback(testblock_name)
            rospy.logdebug(" recorder plugin callback : '%s'", testblock_name)

    def stop_recording(self, testblock_name):
        self.lock.acquire()
        if testblock_name not in self.test.test_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % testblock_name)

        for topic in self.get_topics_of_testblock(testblock_name):
            #print "self.active_topics", self.active_topics
            if topic in self.active_topics.keys():
                self.active_topics[topic]["testblocks"].remove(testblock_name)
            #print "self.active_topics of topic '%s'"%topic, self.active_topics[topic]
            if len(self.active_topics[topic]["testblocks"]) == 0:
                if self.active_topics[topic]["subscriber"] != None:
                    self.active_topics[topic]["subscriber"].unregister()
                self.active_topics.pop(topic)

        #print "<<< stop recording for %s, active_topics="%testblock_name, self.active_topics
        self.lock.release()

    def get_topics_of_testblock(self, testblock_name):
        topics = []
        
        # topics from test_config
        testblock_data = self.test.test_config[testblock_name]
        #print "testblock_data=", testblock_data
        for metric, metric_data in testblock_data.items():
            #print "metric=", metric
            #print "metric_data=", metric_data
            for entry in metric_data:
                # read all "topic" entries
                if "topic" in entry:
                    topic = entry["topic"]
                    if topic not in topics:
                        topics.append(topic)
                        #print "topics==============", topics

        # ask each metric about its topics
        for testblock in self.test.testblocks:
            #print "testblock=", testblock
            for metric_handle in testblock.metric_handles:
                #print "metric_handle=", metric_handle
                for topic in metric_handle.get_topics():
                    #print "topic=", topic
                    if topic not in topics:
                        topics.append(topic)

        # fix global topic prefix
        topics_global = []
        for topic in topics:
            if topic.startswith("/"): # we need to use global topics because rostopic.get_topic_class(topic) can not handle non-global topics
                topics_global.append(topic)
            else:
                topics_global.append("/" + topic)
        #print "topics_global", topics_global
        return topics_global


    
    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        if doc == None:
            doc = {}
        return doc

    def global_topic_callback(self, msg, name):
        self.bag_file_writer.write_to_bagfile(name, msg, rospy.Time.now())

class ATFRecorderError(Exception):
    pass
