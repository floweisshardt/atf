#!/usr/bin/env python
import yaml
import rospkg
import rosparam
import atf_metrics

from atf_core import Testblock


class ATFConfigurationParser:
    def __init__(self):
        self.ns = "/atf/"

        self.parsing_error_message = ""

        # get full config from parameter server (test_config = list of all test configs)
        self.config = rosparam.get_param(self.ns)
        #print "config loader: config=", self.config

        #self.test_name = self.config["test_name"]
        #print "config loader: test_name=\n", self.test_name
        #self.test_config = self.config["test_config"]
        #print "config loader: test_config=\n", self.test_config
        #self.robot_config = self.config["robot_config"]
        #print "config loader: robot_config=\n", self.robot_config

    def get_config(self):
        return self.config

    def create_testblocks(self, config, recorder_handle=None, create_metrics=False):
        testblocks = {}
        for testblock_name in config["test_config"].keys():
            metric_handles = []
            if create_metrics:
                metrics = config["test_config"][testblock_name]
                #print "metrics=", metrics
                metric_handlers_config = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
                #print "metric_handlers_config=", metric_handlers_config
                for metric_name in metrics:
                    #print "metric_name=", metric_name
                    metrics_return_list = getattr(atf_metrics, metric_handlers_config[metric_name]["handler"])().parse_parameter(testblock_name, metrics[metric_name])
                    #print "metrics_return_list=", metrics_return_list
                    if type(metrics_return_list) == list:
                        for metric_return in metrics_return_list:
                            #print "metric_return=", metric_return
                            metric_handles.append(metric_return)
                    else:
                        raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
            #print "metric_handles=", metric_handles
            testblocks[testblock_name] = Testblock(testblock_name, metric_handles, recorder_handle)
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
                    except TypeError as e:
                        raise ATFConfigurationError("TypeError: %s" % str(e))
        return testblock_list

    def load_data(self, filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
            return doc

class ATFConfigurationError(Exception):
    pass
