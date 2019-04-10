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
        #print "recorder_core: self.test:", self.test

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

        # create bag file writer handle
        self.lock_write = Lock()
        self.bag = rosbag.Bag(self.test.generation_config["bagfile_output"] + self.test.name + ".bag", 'w')
        self.bag_file_writer = atf_core.BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        #self.recorder_plugin_list = []
        #if len(recorder_config) > 0:
        #    for value in recorder_config.values():
        #        #print "value=", value
        #        self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.lock_write,
        #                                                                              self.bag_file_writer))

        self.active_topics = {}
        #self.topic_pipeline = []
        #self.active_sections = []
        #self.requested_topics = []
        #self.testblock_list = self.create_testblock_list(test)

        # Wait for obstacle_distance node
        #rospy.loginfo(rospy.get_name() + ": Waiting for obstacle distance node...")
        #ob_sub = rospy.Subscriber("/atf/obstacle_distance", ObstacleDistance, self.global_topic_callback, queue_size=1,
        #                          callback_args="/atf/obstacle_distance")

        #num_subscriber = ob_sub.get_num_connections()
        #while num_subscriber == 0:
        #    num_subscriber = ob_sub.get_num_connections()

        #self.subscribers = []
        #self.topics = ["tf", "robot_status"] 
        #self.topics = self.get_topics()
        #rospy.Timer(rospy.Duration(0.5), self.create_subscriber_callback)

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

    # def create_testblock_list(self, test):
    #     testblock_list = {}
    #     #print "-------------------"
    #     print "test.test_config:", test.test_config
    #     #testblock_list = {'testblock_small': ['/topic1', '/topic2', '/topic3', '/tf'], 'testblock_large': ['/tf', '/topic1', '/topic2', '/topic3']} # FIXME
    #     for testblock_name, testblock_data in test.test_config.items():
    #         print "testblock_name:", testblock_name
    #         print "testblock_data:", testblock_data
    #         for metric_name, metric_data in testblock_data.items():
    #             print "metric_name=", metric_name
    #             print "metric_data=", metric_data
    #             print "robot_config=", test.robot_config
    #             if metric_name in test.robot_config.keys():
    #                 #print "metric is in robot_config"
    #                 try:
    #                     testblock_list[testblock]
    #                 except KeyError:
    #                     testblock_list[testblock] = config["robot_config"][metric]["topics"]
    #                 else:
    #                     for topic in config["robot_config"][metric]["topics"]:
    #                         #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
    #                         if topic[0] != "/":
    #                             topic = "/" + topic
    #                         testblock_list[testblock].append(topic)
    #             else:
    #                 #print "metric is NOT in robot_config"
    #                 try:
    #                     for item in config["robot_config"][metric]:
    #                         #print "item=", item
    #                         if "topic" in item:
    #                             if testblock not in testblock_list:
    #                                 testblock_list.update({testblock: []})
    #                             topic = item['topic']
    #                             #print "topic=", topic
    #                             #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
    #                             if topic[0] != "/":
    #                                 topic = "/" + topic
    #                             testblock_list[testblock].append(topic)
    #                 except TypeError as e:
    #                     raise ATFConfigurationError("TypeError: %s" % str(e))
    #     return testblock_list

    def shutdown(self):
        rospy.loginfo("Shutdown ATF recorder and close bag file.")
        rospy.sleep(1) # let all threads finish writing to bag file
        self.lock_write.acquire()
        self.bag.close()
        self.lock_write.release()

    def add_requested_topics(self, testblock_name):
        #print "add_requested_topics"
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
        self.bag_file_writer.write_to_bagfile("atf/status", status, status.stamp)

    def start_recording(self, testblock_name):
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

        print ">>> start recording for %s, active_topics="%testblock_name, self.active_topics

    def stop_recording(self, testblock_name):
        if testblock_name not in self.test.test_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % testblock_name)

        for topic in self.get_topics_of_testblock(testblock_name):
            #print "self.active_topics", self.active_topics
            if topic in self.active_topics.keys():
                self.active_topics[topic]["testblocks"].remove(testblock_name)
            if len(self.active_topics[topic]["testblocks"]) == 0:
                if self.active_topics[topic]["subscriber"] != None:
                    self.active_topics[topic]["subscriber"].unregister()
                self.active_topics.pop(topic)

        #print "<<< stop recording for %s, active_topics="%testblock_name, self.active_topics

    def get_topics_of_testblock(self, testblock_name):
        topics = []
        
        # topics from test_config
        testblock_data = self.test.test_config[testblock_name]
        #print "testblock_data=", testblock_data
        for metric, metric_data in testblock_data.items():
            #print "metric=", metric
            #print "metric_data=", metric_data
            for entry in metric_data:
                if "topic" in entry:
                    topic = entry["topic"]
                    if topic not in topics:
                        topics.append(topic)
                        #print "topics==============", topics

        # topics from robot_config
        #print "self.test.robot_config", self.test.robot_config
        for topic in self.test.robot_config["topics"]:
            if topic not in topics:
                topics.append(topic)

        #print "topics", topics

        # fix global topic prefix
        topics_global = []
        for topic in topics:
            if topic.startswith("/"): # we need to use global topics because rostopic.get_topic_class(topic) can not handle non-global topics
                topics_global.append(topic)
            else:
                topics_global.append("/" + topic)
        #print "topics_global", topics_global
        return topics_global


    def record_trigger(self, trigger):
        print "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrecord_trigger:", trigger
        #print "self.test.test_config", self.test.test_config

        if trigger.name not in self.test.test_config:
            raise ATFRecorderError("Testblock '%s' not in test config" % trigger.name)

        # Send message to all recorder plugins
        print "self.recorder_plugin_list=", self.recorder_plugin_list
        for recorder_plugin in self.recorder_plugin_list:
        #    #FIXME: need to filter the topics not needed for current trigger
            recorder_plugin.trigger_callback(trigger)
            rospy.loginfo(" =!=!=!=!=!=!=!=!=!=recorder plugin callback : '%s'", trigger.name)

        # Only process message if testblock requests topics
        print "ttttttttttttttttttttttttt self.testblock_list=", self.testblock_list
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

        self.bag_file_writer.write_to_bagfile(self.ns + trigger.name + "/trigger", trigger, trigger.stamp)


    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        if doc == None:
            doc = {}
        return doc

    def global_topic_callback(self, msg, name):
        self.bag_file_writer.write_to_bagfile(name, msg, rospy.Time.now())

    def get_topics(self):
        topics = []
        #print "self.testblock_list=", self.testblock_list
        #print "self.test.test_config=", self.test.test_config
        for testblock, testblock_data in self.test.test_config.items():
            #print "testblock=", testblock
            #print "testblock_data=", testblock_data
            for metric, metric_data in testblock_data.items():
                #print "metric=", metric
                #print "metric_data=", metric_data
                for entry in metric_data:
                    if "topic" in entry:
                        if entry["topic"] not in topics:
                            topics.append(entry["topic"])
                            #print "topics==============", topics
        return topics

class ATFRecorderError(Exception):
    pass
