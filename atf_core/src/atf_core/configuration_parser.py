#!/usr/bin/env python
import atf_core
import yaml
import rospkg
import rosparam
import atf_metrics
import os
import itertools as it
import json

from atf_core import Test, Testblock

class ATFConfigurationParser:
    def __init__(self, package_name, test_generation_config_file = None, recorder_handle = None):
        if test_generation_config_file == None:
            test_generation_config_file = "atf/test_generation_config.yaml"
            print "ATF Warning: No test_generation_config_file specified. Continue using default '%s'"%test_generation_config_file

        self.parsing_error_message = ""
        
        self.generation_config = self.load_data(os.path.join(rospkg.RosPack().get_path(package_name), test_generation_config_file))
        #print "generation_config:", self.generation_config

        # check for required parameters
        try:
            self.generation_config["tests_config_path"]
            self.generation_config["robots_config_path"]
            self.generation_config["envs_config_path"]
            self.generation_config["testblocksets_config_path"]
            self.generation_config["app_executable"]
            self.generation_config["app_launch_file"]
            self.generation_config["bagfile_output"]
            self.generation_config["txt_output"]
            self.generation_config["json_output"]
            self.generation_config["yaml_output"]
            self.generation_config["testsuites"]
        except KeyError as e:
            error_message = "ATF Error: parsing test configuration failed. Missing Key: " + str(e)
            print error_message
            raise ATFConfigurationError(error_message)

        # check for optional parameters
        keys = [("time_limit_recording", 60.0),
                ("time_limit_analysing", 60.0),
                ("time_limit_uploading", 60.0),
                ("upload_data", False),
                ("upload_result", False)]
        
        for key, default_value in keys:
            if key not in self.generation_config.keys():
                self.generation_config[key] = default_value
                print "ATF Warning: parsing test configuration incomplete, missing key '%s'. Continuing with default value of %s."%(key, str(self.generation_config[key]))

        self.tests = []
        self.test_list = []
        testsuite_id = 0
        test_config_id = 0
        robot_id = 0
        env_id = 0
        testblockset_id = 0
        for testsuite_name in self.generation_config["testsuites"].keys():
            #print "testsuite:", testsuite_name
            for test_config_name in self.generation_config["testsuites"][testsuite_name]["tests"]:
                #print "test_config:", test_config_name
                for robot_name in self.generation_config["testsuites"][testsuite_name]["robots"]:
                    #print "robot:", robot_name
                    for env_name in self.generation_config["testsuites"][testsuite_name]["envs"]:
                        #print "robot_env:", env_name
                        for testblockset_name in self.generation_config["testsuites"][testsuite_name]["testblocksets"]:
                            #print "testblocks:", testblocks_name
                            test_group_name = "ts" + str(testsuite_id) + "_c" + str(test_config_id) + "_r" + str(robot_id) + "_e" + str(env_id) + "_s" + str(testblockset_id)
                            test_list_element = {}
                            test_list_element[test_group_name] = {}
                            test_list_element[test_group_name]["subtests"] = []
                            for repetition in range(0,self.generation_config["testsuites"][testsuite_name]["repetitions"]):
                                name = test_group_name + "_" + str(repetition)
                                test = Test()
                                test.package_name = package_name
                                test.name = name
                                test.generation_config = self.generation_config
                                test.testsuite_name = testsuite_name
                                test.testsuite = None
                                test.test_config_name = test_config_name
                                test.test_config = self.load_data(os.path.join(rospkg.RosPack().get_path(test.package_name), self.generation_config["tests_config_path"], test_config_name + ".yaml"))
                                test.robot_name = robot_name
                                test.robot_config = self.load_data(os.path.join(rospkg.RosPack().get_path(test.package_name), self.generation_config["robots_config_path"], robot_name + ".yaml"))
                                test.env_name = env_name
                                test.env_config = self.load_data(os.path.join(rospkg.RosPack().get_path(test.package_name), self.generation_config["envs_config_path"], env_name + ".yaml"))
                                test.testblockset_name = testblockset_name
                                test.testblockset_config = self.load_data(os.path.join(rospkg.RosPack().get_path(test.package_name), self.generation_config["testblocksets_config_path"], testblockset_name + ".yaml"))
                                
                                #test.print_to_terminal()
                                #print test.name
                                
                                test.metrics = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
                                test.testblocks = []
                                for testblock_name in test.testblockset_config.keys():
                                    if recorder_handle != None:
                                        metric_handles = None
                                    else:
                                        metric_handles = self.create_metric_handles(test, testblock_name, True)
                                    #print "metric_handles", metric_handles
                                    testblock = Testblock(testblock_name, metric_handles, recorder_handle)
                                    test.testblocks.append(testblock)
                                
                                self.tests.append(test)
                                test_list_element[test_group_name]["subtests"].append(test.name)
                            
                        
                            test_list_element[test_group_name]["robot"] = test.robot_name
                            test_list_element[test_group_name]["robot_env"] = test.env_name
                            test_list_element[test_group_name]["test_config"] = test.test_config_name
                            test_list_element[test_group_name]["testblockset"] = test.testblockset_name
                            self.test_list.append(test_list_element)
                            testblockset_id += 1
                        env_id += 1
                        testblockset_id = 0
                    robot_id += 1
                    env_id = 0
                test_config_id += 1
                robot_id = 0
            testsuite_id += 1
            test_config_id = 0
        #print "number of tests:", len(self.tests)

    def get_tests(self):
        return self.tests
    
    def get_test_list(self):
        return self.test_list
    
    def export_to_file(self, data, target):
        if not os.path.exists(os.path.dirname(target)):
            os.makedirs(os.path.dirname(target))
        stream = file(target, 'w')
        file_extension = os.path.splitext(target)[1]
        if file_extension == ".json": # get file extension
            json.dump(data, stream)
        elif file_extension == ".yaml": # get file extension
            yaml.dump(data, stream, default_flow_style=False)
        elif file_extension == ".txt":
            stream.write(str(data))
        else:
            raise ATFConfigurationError("ATF cannot export file extension %s"%(file_extension))

    def create_metric_handles(self, test, testblock_name, create_metrics):
        metric_handles = []
        if create_metrics:
            metrics = test.testblockset_config[testblock_name]
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

    def load_data(self, filename):
        #print "config parser filename:", filename
        if os.path.isfile(filename):
            with open(filename, 'r') as stream:
                doc = yaml.load(stream)
                return doc
        else:
            error_message = "ATF Error: file not found: %s"%filename
            print error_message
            raise ATFConfigurationError(error_message)

class ATFConfigurationError(Exception):
    pass
