#!/usr/bin/env python
import rospy
import yaml
import os
import unittest
import rostest
import rospkg
import rosparam

from atf_core import Testblock
import atf_metrics


class ATFConfigurationParser:
    def __init__(self):
        self.ns = "/atf/"

        self.parsing_error_message = ""

        # get full config from parameter server (test_config = list of all test configs)
        self.config = rosparam.get_param(self.ns)
        # select the current test_config and replace test_config (list) with the current test_config (test_config[test_config_name])
        self.config["test_config"] = self.config["test_config"][self.config["test_config_name"]] 
        #print "config loader: config=", self.config

        #self.test_name = self.config["test_name"]
        #print "config loader: test_name=\n", self.test_name
        #self.test_config = self.config["test_config"]
        #print "config loader: test_config=\n", self.test_config
        #self.robot_config = self.config["robot_config"]
        #print "config loader: robot_config=\n", self.robot_config

    def get_config(self):
        return self.config

    def create_testblocks(self, config, recorder_handle):
        testblocks = {}
        for testblock_name in config["test_config"].keys():
            metrics = config["test_config"][testblock_name]
            #print "metrics=", metrics
            testblocks[testblock_name] = Testblock(testblock_name, [], recorder_handle)
        return testblocks

    def create_testblock_list(self, config):
        testblock_list = {}
        #print "-------------------"
        for testblock in config["test_config"].keys():
            #print "testbock=", testblock
            for metric in config["test_config"][testblock].keys():
                #print "metric=", metric
                #print "robot_config=", config["robot_config"]
                if metric in config["robot_config"]:
                    #print "metric is in robot_config"
                    try:
                        testblock_list[testblock]
                    except KeyError:
                        testblock_list[testblock] = config["robot_config"][metric]["topics"]
                    else:
                        for topic in config["robot_config"][metric]["topics"]:
                            #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
                            if topic[0] != "/":
                                topic = "/" + topic
                            testblock_list[testblock].append(topic)
                else:
                    #print "metric is NOT in robot_config"
                    try:
                        for item in config["test_config"][testblock][metric]:
                            print "item=", item
                            if "topic" in item:
                                if testblock not in testblock_list:
                                    testblock_list.update({testblock: []})
                                topic = item['topic']
                                #print "topic=", topic
                                #add heading "/" to all topics to make them global (rostopic.get_topic_class() cannot handle non global topics)
                                if topic[0] != "/":
                                    topic = "/" + topic
                                testblock_list[testblock].append(topic)
                    except TypeError as e:
                        print "TypeError: %s" % str(e)
                        pass
        return testblock_list

    def get_test_list(self):
        test_config_path = rospy.get_param(self.ns + "test_config_file")
        config_data = self.load_data(test_config_path)[rospy.get_param(self.ns + "test_config")]
        #metrics_data = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")

        testblock_list = []

        for testblock_name in config_data:
            metrics = []

            for metric in config_data[testblock_name]:
                metric_return = getattr(atf_metrics, metrics_data[metric]["handler"])().parse_parameter(testblock_name, config_data[testblock_name][metric])
                if type(metric_return) == list:
                    for metric in metric_return:
                        metrics.append(metric)
                else:
                    self.parsing_error_message = "no valid metric configuration for metric '" + metric + "' in testblock '" + testblock_name + "'"
                    rospy.logerr(self.parsing_error_message)
                    return False

            testblock_list.append(Testblock(testblock_name, metrics))

        return testblock_list

    def load_data(self, filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
            return doc
