#!/usr/bin/env python
import fnmatch
import yaml
import rospkg
import os
import json
import rosbag

from atf_metrics.error import ATFConfigurationError
from atf_core.test import Test
from atf_core.testblock import Testblock
import atf_metrics

class ATFConfigurationParser:
    def __init__(self, package_name = None, test_generation_config_file = None, skip_metrics = False):
        if package_name == None: # no package given
            return
        elif "/" in package_name: # assume no package but full path to package (needed for generate_tests.py in travis because RPS_PACKAGE_PATH is not yet set correclty)
            full_path_to_test_package = package_name
        else: # assume package name only
            full_path_to_test_package = rospkg.RosPack().get_path(package_name)

        if test_generation_config_file == None:
            test_generation_config_file = "atf/test_generation_config.yaml"
            print("ATF Warning: No test_generation_config_file specified. Continue using default '%s'"%test_generation_config_file)

        self.generation_config = self.load_data(os.path.join(full_path_to_test_package, test_generation_config_file))
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
            print(error_message)
            raise ATFConfigurationError(error_message)

        # check for optional parameters
        keys = [("time_limit_recording", 60.0),
                ("time_limit_analysing", 60.0),
                ("time_limit_uploading", 60.0),
                ("upload_data", False),
                ("upload_result", False)]
        for key, default_value in keys:
            if key not in list(self.generation_config.keys()):
                self.generation_config[key] = default_value
                print("ATF Warning: parsing test configuration incomplete, missing key '%s'. Continuing with default value of %s."%(key, str(self.generation_config[key])))

        self.tests = []
        self.test_list = []
        testsuite_id = 0
        test_config_id = 0
        robot_id = 0
        env_id = 0
        testblockset_id = 0
        #print self.generation_config["testsuites"]  
        for testsuite in self.generation_config["testsuites"]:
            #print "testsuite:", testsuite
            for test_config_name in testsuite["tests"]:
                #print "test_config:", test_config_name
                for robot_name in testsuite["robots"]:
                    #print "robot:", robot_name
                    for env_name in testsuite["envs"]:
                        #print "robot_env:", env_name
                        for testblockset_name in testsuite["testblocksets"]:
                            #print "testblocks:", testblocks_name
                            test_group_name = "ts" + str(testsuite_id) + "_c" + str(test_config_id) + "_r" + str(robot_id) + "_e" + str(env_id) + "_s" + str(testblockset_id)
                            test_list_element = {}
                            test_list_element[test_group_name] = {}
                            test_list_element[test_group_name]["subtests"] = []
                            # repetitions is an optional parameter, default = 1
                            if "repetitions" not in testsuite:
                                testsuite["repetitions"] = 1
                            for repetition in range(0,testsuite["repetitions"]):
                                name = test_group_name + "_" + str(repetition)
                                test = Test()
                                test.package_name = package_name
                                test.name = name
                                test.generation_config = self.generation_config
                                test.testsuite = None
                                test.test_config_name = test_config_name
                                test.test_config = self.load_data(os.path.join(full_path_to_test_package, self.generation_config["tests_config_path"], test_config_name + ".yaml"))
                                self.parse_key_as_list(test.test_config, "additional_parameters")
                                self.parse_key_as_list(test.test_config, "additional_arguments")
                                test.robot_name = robot_name
                                test.robot_config = self.load_data(os.path.join(full_path_to_test_package, self.generation_config["robots_config_path"], robot_name + ".yaml"))
                                self.parse_key_as_list(test.robot_config, "additional_parameters")
                                self.parse_key_as_list(test.robot_config, "additional_arguments")
                                test.env_name = env_name
                                test.env_config = self.load_data(os.path.join(full_path_to_test_package, self.generation_config["envs_config_path"], env_name + ".yaml"))
                                self.parse_key_as_list(test.env_config, "additional_parameters")
                                self.parse_key_as_list(test.env_config, "additional_arguments")
                                test.testblockset_name = testblockset_name
                                test.testblockset_config = self.load_data(os.path.join(full_path_to_test_package, self.generation_config["testblocksets_config_path"], testblockset_name + ".yaml"))

                                #test.print_to_terminal()
                                #print test.name
                                
                                if not skip_metrics:
                                    test.metrics = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
                                    test.testblocks = []
                                    for testblock_name in list(test.testblockset_config.keys()):
                                        metric_handles = self.create_metric_handles(test, testblock_name, True)
                                        #print "metric_handles", metric_handles
                                        testblock = Testblock(testblock_name, metric_handles, None)
                                        test.testblocks.append(testblock)
                                else:
                                    print("ATF: skip_metrics is set. Skipping metric and testblock configuration.")
                                
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
        stream = open(target, 'w')
        file_extension = os.path.splitext(target)[1]
        if file_extension == ".json": # get file extension
            json.dump(data, stream)
        elif file_extension == ".yaml": # get file extension
            yaml.dump(data, stream, default_flow_style=False)
        elif file_extension == ".txt":
            stream.write(str(data))
        elif file_extension == ".bag":
            bag = rosbag.Bag(target, 'w')
            bag.write("atf_result", data)
            bag.close()
        else:
            raise ATFConfigurationError("ATF cannot export file extension %s"%(file_extension))

    def create_metric_handles(self, test, testblock_name, create_metrics):
        metric_handles = []
        if create_metrics:
            metrics = test.testblockset_config[testblock_name]
            #print "metrics=", metrics
            metric_handlers_config = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")
            #print "metric_handlers_config=", metric_handlers_config
            if metric_handlers_config and metrics:
                for metric_type in list(metrics.keys()):
                    if metric_type not in metric_handlers_config:
    	                raise ATFConfigurationError("metric '%s' is not implemented"%metric_type)

                    if len(metrics[metric_type]) == 0:
                        raise ATFConfigurationError("empty configuration for metric '%s' in testblock '%s' (should be a list of dicts, e.g. '[{}]' for no parameters)"%(metric_type, testblock_name))

                    for id, params in enumerate(metrics[metric_type]):
                        if not self.validate_metric_parameters(metric_type, params):
                            raise ATFConfigurationError("invalid configuration for metric '%s' in testblock '%s': %s"%(metric_type, testblock_name, str(params)))

                        try:
                            suffix = params["suffix"]
                        except (TypeError, KeyError):
                            suffix = id
                        metric_name = metric_type + "::" + str(suffix)

                        # check if metric_handle.names are unique #FIXME is there a more performant way to implement this without the additional for loop?
                        for metric_handle in metric_handles:
                            if metric_name == metric_handle.name:
                                raise ATFConfigurationError("metric_name '%s' is not unique in testblock '%s'"%(metric_name, testblock_name))

                        metric_handle = getattr(atf_metrics, metric_handlers_config[metric_type]["handler"])().parse_parameter(testblock_name, metric_name, params)
                        metric_handles.append(metric_handle)

        #print "metric_handles=", metric_handles
        return metric_handles

    def validate_metric_parameters(self, metric_type, params):
        if params == None:
            print("params None")
            return False

        if type(params) is not dict:
            print("params not a dict")
            return False        

        try:
            params["groundtruth"]
        except (TypeError, KeyError):
            return True # no groundtruth specified

        try:
            params["groundtruth"]["data"]
            params["groundtruth"]["epsilon"]
            return True # groundtruth specified
        except (TypeError, KeyError):
            # e.g. (params["groundtruth"]["data"] == None and params["groundtruth"]["epsilon"] != None) or (params["groundtruth"]["data"] != None and params["groundtruth"]["epsilon"] == None)
            print("invalid groundtruth specified:", params)
            return False

        return True # all checks successfull

    def load_data(self, filename):
        #print "config parser filename:", filename
        if os.path.isfile(filename):
            with open(filename, 'r') as stream:
                doc = yaml.safe_load(stream)
                return doc
        else:
            error_message = "ATF Error: file not found: %s"%filename
            print(error_message)
            raise ATFConfigurationError(error_message)

    def parse_key_as_list(self, dictionary, key):
        #print "dict=\n", dictionary
        if dictionary != None and key in list(dictionary.keys()):
            value = dictionary[key]
            value_type = type(dictionary[key])
            # make sure all additional_parameters and additional_arguments are lists
            if type(value) == list:
                for element in value:
                    if type(element) == dict:
                        return
            elif type(value) == dict:
                dictionary[key] = [dictionary[key]]
                for element in dictionary[key]:
                    if type(element) == dict:
                        return
        
            error_message = "ATF configuration Error: key '%s' of type '%s' with value '%s' cannot be parsed as list of dictionaries"%(str(key), value_type, value)
            print(error_message)
            raise ATFConfigurationError(error_message)

    def match_filter(self, name, filter):
        if filter == "":
            return True

        filter_list = filter.split(',')
        for filter in filter_list:
            if fnmatch.fnmatch(name, filter):
                return True
        
        return False

    def get_sorted_plot_dicts(self, atf_result, filter_tests, filter_testblocks, filter_metrics):
        tbm = {}
        tmb = {}
        bmt = {}
        mbt = {}
        mtb = {}

        for test in atf_result.results:
            #print test.name
            if not self.match_filter(test.name, filter_tests):
                continue

            test_description = "(%s, %s, %s, %s)"%(test.test_config, test.robot, test.env, test.testblockset)

            for testblock in test.results:
                #print "  -", testblock.name
                if not self.match_filter(testblock.name, filter_testblocks):
                    continue

                for metric in testblock.results:
                    #print "    -", metric.name
                    if not self.match_filter(metric.name, filter_metrics):
                        continue

                    # use metric_name with unit
                    metric_name = metric.name + "\n[" + metric.unit + "]"
                    metric_name_tbm = metric.name + " [" + metric.unit + "]"

                    # tbm
                    if test.name                 not in list(tbm.keys()):
                        tbm[test.name] = {}
                    if testblock.name            not in list(tbm[test.name].keys()):
                        tbm[test.name][testblock.name] = {}
                    tbm[test.name][testblock.name][metric_name_tbm] = metric

                    # tmb
                    if test.name                 not in list(tmb.keys()):
                        tmb[test.name] = {}
                    if metric_name               not in list(tmb[test.name].keys()):
                        tmb[test.name][metric_name] = {}
                    tmb[test.name][metric_name][testblock.name] = metric

                    # bmt
                    if testblock.name            not in list(bmt.keys()):
                        bmt[testblock.name] = {}
                    if metric_name               not in list(bmt[testblock.name].keys()):
                        bmt[testblock.name][metric_name] = {}
                    bmt[testblock.name][metric_name][test.name] = metric

                    # mbt
                    if metric_name            not in list(mbt.keys()):
                        mbt[metric_name] = {}
                    if testblock.name         not in list(mbt[metric_name].keys()):
                        mbt[metric_name][testblock.name] = {}
                    mbt[metric_name][testblock.name][test.name + "\n" + test_description] = metric

                    # mtb
                    if metric_name            not in list(mtb.keys()):
                        mtb[metric_name] = {}
                    #test_name = test.name
                    test_name = test.name + "\n" + test.robot
                    if test_name              not in list(mtb[metric_name].keys()):
                        mtb[metric_name][test_name] = {}
                    mtb[metric_name][test_name][testblock.name] = metric

        ret = {}
        ret['tbm'] = tbm
        ret['tmb'] = tmb
        ret['bmt'] = bmt
        ret['mbt'] = mbt
        ret['mtb'] = mtb
        return ret
