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
    def __init__(self, config, testblock_list):
        self.ns = "/atf/"
        self.config = config
        #print "recorder_core: config=", self.config

        recorder_config = self.load_data(rospkg.RosPack().get_path("atf_recorder_plugins") +
                                         "/config/recorder_plugins.yaml")

        try:
            # delete test_results directories and create new ones
            if os.path.exists(self.config["bagfile_output"]):
            #    shutil.rmtree(self.config["bagfile_output"]) #FIXME will fail if multiple test run concurrently
                pass
            else:
                os.makedirs(self.config["bagfile_output"])
        except OSError:
            pass

        # create bag file writer handle
        self.lock_write = Lock()
        self.bag = rosbag.Bag(config["bagfile_output"] + self.config["test_name"] + ".bag", 'w')
        self.bag_file_writer = atf_core.BagfileWriter(self.bag, self.lock_write)

        # Init metric recorder
        self.recorder_plugin_list = []
        if len(recorder_config) > 0:
            for value in recorder_config.values():
                #print "value=", value
                self.recorder_plugin_list.append(getattr(atf_recorder_plugins, value)(self.lock_write,
                                                                                      self.bag_file_writer))

        self.topic_pipeline = []
        self.active_sections = []
        self.requested_topics = []
        self.testblock_list = testblock_list

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
        for topic in self.config["robot_config"]["wait_for_topics"]:
            rospy.loginfo("Waiting for topic '%s'...", topic)
            rospy.wait_for_message(topic, rospy.AnyMsg)
            rospy.loginfo("... got message on topic '%s'.", topic)

        for service in self.config["robot_config"]["wait_for_services"]:
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

        #test_status = TestStatus()
        #test_status.test_name = self.bag_name
        #test_status.status_recording = 3
        #test_status.status_analysing = 0
        #test_status.total = self.number_of_tests

        #self.test_status_publisher.publish(test_status)

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
                except Exception:
                    pass

    def record_trigger(self, trigger):
        #print "record_trigger: name=", trigger.name, "trigger=", trigger.trigger, "stamp=", trigger.stamp

        if trigger.name not in self.config["test_config"]:
            raise ATFRecorderError("Testblock '%s' not in test config" % trigger.name)

        # Send message to all recorder plugins
        #print "self.recorder_plugin_list=", self.recorder_plugin_list
        for recorder_plugin in self.recorder_plugin_list:
            recorder_plugin.trigger_callback(trigger)

        # Only process message if testblock requests topics
        #print "self.testblock_list=", self.testblock_list
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
                                  trigger.stamp)


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

class ATFRecorderError(Exception):
    pass
