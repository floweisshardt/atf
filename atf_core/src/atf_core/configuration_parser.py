#!/usr/bin/env python
import yaml
import rospkg
import rosparam
import atf_metrics
import os

from atf_core import Test, Testblock

class ATFConfigurationParser:
    def __init__(self):
        self.ns = "/atf/"

        self.parsing_error_message = ""

        # get full config from parameter server (test_config = list of all test configs)
        self.config = rosparam.get_param(self.ns)
        
        testsuites = self.load_data(rospkg.RosPack().get_path("atf_test_app_time") + "/config/test_suites.yaml")
        testgeneration = self.load_data(rospkg.RosPack().get_path("atf_test_app_time") + "/config/test_generation_config.yaml")
        #print "testsuites:", testsuites
        #print "testgeneration:", testgeneration
        self.tests = []
        testsuite_id = 0
        test_config_id = 0
        robot_id = 0
        robot_env_id = 0
        for testsuite_name in testsuites.keys(): # TODO fix package name
            #print "testsuite:", testsuite_name
            for test_config_name in testsuites[testsuite_name]["test_configs"]:
                #print "test_config:", test_config_name
                for robot_name in testsuites[testsuite_name]["robots"]:
                    #print "robot:", robot_name
                    for robot_env_name in testsuites[testsuite_name]["robot_envs"]:
                        #print "robot_env:", robot_env_name
                        for repetition in range(0,testgeneration["repetitions"]):
                            name = "ts" + str(testsuite_id) + "_c" + str(test_config_id) + "_r" + str(robot_id) + "_e" + str(robot_env_id) + "_" + str(repetition)
                            #print name
                            test = Test()
                            test.name = name
                            test.testsuite_name = testsuite_name
                            test.testsuite = None
                            test.test_config_name = test_config_name
                            test.test_config = self.load_data(rospkg.RosPack().get_path("atf_test_app_time") + "/config/test_configs/" + test_config_name + ".yaml")
                            test.robot_name = robot_name
                            test.robot = self.load_data(rospkg.RosPack().get_path("atf_test_app_time") + "/config/robots/" + robot_name + ".yaml")
                            test.robot_env_name = robot_env_name
                            test.robot_env = self.load_data(rospkg.RosPack().get_path("atf_test_app_time") + "/config/robot_envs/" + robot_env_name + ".yaml")
                            test.generation_config = testgeneration
                            
                            #test.print_to_terminal()
                            #print test.name
                            
                            test.metrics = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
                            test.testblocks = []
                            for testblock_name in test.test_config.keys():
                                #print testblock_name
                                metric_handles = self.create_metric_handles(test, testblock_name, True)
                                #print "metric_handles", metric_handles
                                testblock = Testblock(testblock_name, metric_handles, None)
                                test.testblocks.append(testblock)
                            
                            self.tests.append(test)
                        robot_env_id += 1
                    robot_id += 1
                    robot_env_id = 0
                test_config_id += 1
                robot_id = 0
            testsuite_id += 1
        print "number of tests:", len(self.tests)
        
        
        #print "config loader: config=", self.config

        #self.test_name = self.config["test_name"]
        #print "config loader: test_name=\n", self.test_name
        #self.test_config = self.config["test_config"]
        #print "config loader: test_config=\n", self.test_config
        #self.robot_config = self.config["robot_config"]
        #print "config loader: robot_config=\n", self.robot_config

    def get_config(self):
        return self.config
    
    def get_tests(self):
        return self.tests

    def create_metric_handles(self, test, testblock_name, create_metrics):
        metric_handles = []
        if create_metrics:
            metrics = test.test_config[testblock_name]
            #print "metrics=", metrics
            metric_handlers_config = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
            #print "metric_handlers_config=", metric_handlers_config
            if  metric_handlers_config and metrics:
                for metric_name in metrics:
                    #print "metric_name=", metric_name
                    metrics_return_list = getattr(atf_metrics, metric_handlers_config[metric_name]["handler"])().parse_parameter(testblock_name, metrics[metric_name])
                    #print "metrics_return_list=", metrics_return_list
                    if metrics_return_list and (type(metrics_return_list) == list):
                        for metric_return in metrics_return_list:
                            #print "metric_return=", metric_return
                            metric_handles.append(metric_return)
                    else:
                        raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
        #print "metric_handles=", metric_handles
        return metric_handles

    #def create_testblocks(self, config, recorder_handle=None, create_metrics=False):
    #    testblocks = {}
    #    for testblock_name in config["test_config"].keys():
    #        metric_handles = self.create_metric_handles(create_metrics)
    #        testblocks[testblock_name] = Testblock(testblock_name, metric_handles, recorder_handle)
    #    return testblocks

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
        #print "config parser filename:", filename
        if os.path.isfile(filename):
            with open(filename, 'r') as stream:
                doc = yaml.load(stream)
                return doc

class ATFConfigurationError(Exception):
    pass
