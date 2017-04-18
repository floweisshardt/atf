#!/usr/bin/env python
import yaml
import json
import re
import itertools as it
import rospkg
import os
import lxml.etree
import lxml.builder
import shutil
import sys

from xml.etree import ElementTree
from xml.dom import minidom

from copy import deepcopy, copy


class GenerateTests:
    def __init__(self, arguments):
        self.ns = "/atf/"
        self.print_output = "ATF: Test generation done!"
        self.package_name = arguments[1]
        self.package_src_path = arguments[2]
        self.package_bin_path = arguments[3]
        self.generation_config = self.load_yaml(self.package_src_path + "/config/test_generation_config.yaml")

        # required parameters
        try:
            self.generation_config["suites_file"]
            self.generation_config["time_limit_recording"]
            self.generation_config["time_limit_analysing"]
            self.generation_config["time_limit_uploading"]
            self.generation_config["bagfile_output"]
            self.generation_config["json_output"]
            self.generation_config["yaml_output"]
        except KeyError as e:
            error_message = "Error: parsing test configuration failed. Missing Key: " + str(e)
            print error_message
            self.print_output = "ATF: Test generation failed! " + error_message
            sys.exit(1)

        # optional parameters
        try:
            if "repetitions" not in self.generation_config:
                self.generation_config["repetitions"] = 1
            if type(self.generation_config["upload_data"]) is bool:
                self.upload_data = self.generation_config["upload_data"]
            else:
                self.upload_data = False
            if type(self.generation_config["upload_result"]) is bool:
                self.upload_result = self.generation_config["upload_result"]
            else:
                self.upload_result = False
            if "speed_factor_analysis" in self.generation_config:
                self.speed_factor_analysis = self.generation_config["speed_factor_analysis"]
            else:
                self.speed_factor_analysis = 1
        except KeyError as e:
            error_message = "ATF: Warning: parsing test configuration incomplete. Missing Key: " + str(e)
            print error_message
            self.upload_data = False
            self.upload_result = False

        self.test_list = {}

        self.test_generated_path = os.path.join(self.package_bin_path, "test_generated")
        self.test_generated_recording_path = os.path.join(self.test_generated_path, "recording")
        self.test_generated_analysing_path = os.path.join(self.test_generated_path, "analysing")
        self.create_folders()
        self.test_list = self.generate_test_list()

    def create_folders(self):
        # delete of test_generated directory and create new one
        if os.path.exists(self.test_generated_recording_path):
            shutil.rmtree(self.test_generated_path)
        shutil.copyfile(self.package_src_path + "/package.xml", self.package_bin_path + "/package.xml")
        os.makedirs(self.test_generated_path)
        os.makedirs(self.test_generated_recording_path)
        os.makedirs(self.test_generated_analysing_path)

    def generate_tests(self):
        em = lxml.builder.ElementMaker()
        launch = em.launch
        arg = em.arg
        include = em.include
        test = em.test
        node = em.node
        param = em.param
        rosparam = em.rosparam

        #print "self.test_list=", self.test_list
        for test_name in self.test_list.keys():
            #print "\ntest_name=", test_name
            test_config = self.load_yaml(os.path.join(self.package_src_path, self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml"))
            robot_config = self.load_yaml(os.path.join(self.package_src_path, self.generation_config["robot_config_path"], self.test_list[test_name]["robot"] + ".yaml"))
            robot_env_config = self.load_yaml(os.path.join(self.package_src_path, self.generation_config["robot_env_config_path"], self.test_list[test_name]["robot_env"] + ".yaml"))
            #print "robot_config=", robot_config
            #print "robot_env_config=", robot_env_config
            #print "self.test_list[test_name]=", self.test_list[test_name]

            # Cleaning
            test_clean = launch(
                param(name=self.ns + "bag_output", value=self.generation_config["bagfile_output"]),
                param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
                param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
                test({'test-name': "cleaning", 'pkg': "atf_core", 'type': "cleaner.py",
                      'time-limit': "10"})
            )
            xmlstr = minidom.parseString(ElementTree.tostring(test_clean)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "cleaning.test")
            with open(filepath, "w") as f:
                f.write(xmlstr)

            for subtest_name in self.test_list[test_name]["subtests"]:
                # Recording
                test_record = launch(
                    #arg(name="robot", value=self.test_list[test_name]["robot"]),
                    #include(arg(name="test_status_list", value="$(find " + self.package_name + ")/test_status.yaml"),
                    #        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                    param(name=self.ns + "test_name", value=subtest_name),
                    param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                    param(name=self.ns + "robot_config_name", value=self.test_list[test_name]["robot"]),
                    param(name=self.ns + "robot_env_config_name", value=self.test_list[test_name]["robot_env"]),
                    rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml")),
                    rosparam(param=self.ns + "robot_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["robot_config_path"], self.test_list[test_name]["robot"] + ".yaml")),
                    rosparam(param=self.ns + "robot_env_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["robot_env_config_path"], self.test_list[test_name]["robot_env"] + ".yaml")),
                    param(name=self.ns + "bagfile_output", value=self.generation_config["bagfile_output"]),
                    param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
                    param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
                )

                if "additional_launch_file" in self.generation_config:
                    incl = include(file="$(find " + self.package_name + ")/" + self.generation_config["additional_launch_file"])
                    # robot args and params
                    if "additional_arguments" in robot_config:
                        if len(robot_config["additional_arguments"]) > 0:
                            for robot_arg_name, robot_arg_value in robot_config["additional_arguments"].items():
                                incl.append(arg(name=str(robot_arg_name), value=str(robot_arg_value)))
                    if "additional_parameters" in robot_config:
                        if len(robot_config["additional_parameters"]) > 0:
                            for robot_param_name, robot_param_value in robot_config["additional_parameters"].items():
                                incl.append(param(name=str(robot_param_name), value=str(robot_param_value)))
                    # robot_env args and params
                    if "additional_arguments" in robot_env_config:
                        if len(robot_env_config["additional_arguments"]) > 0:
                            for robot_env_arg_name, robot_env_arg_value in robot_env_config["additional_arguments"].items():
                                incl.append(arg(name=str(robot_env_arg_name), value=str(robot_env_arg_value)))
                    if "additional_parameters" in robot_env_config:
                        if len(robot_env_config["additional_arguments"]) > 0:
                            for robot_env_param_name, robot_env_param_value in robot_env_config["additional_parameters"].items():
                                incl.append(param(name=str(robot_env_param_name), value=str(robot_env_param_value)))
                    test_record.append(incl)

                test_record.append(test({'test-name': "recording_" + subtest_name, 'pkg': self.package_name, 'type': self.generation_config['app_executable'],
                          'time-limit': str(self.generation_config["time_limit_recording"]), 'required': "true"}))

                xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
                filepath = os.path.join(self.test_generated_recording_path, "recording_" + subtest_name) + ".test"
                with open(filepath, "w") as f:
                    f.write(xmlstr)

                # Analysing
                test_analyse = launch(
                    #include(arg(name="test_status_list", value="$(find " + self.package_name + ")/test_status.yaml"),
                    #        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                    param(name=self.ns + "test_name", value=subtest_name),
                    param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                    rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml")),
                    #param(name="test_config", value=self.test_list[test_name]["test_config"]),
                    #param(name="test_config_file", value="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                    param(name=self.ns + "test_generated_path", value=self.test_generated_path),
                    #param(name="yaml_output", value=self.yaml_output),
                    #param(name="json_output", value=self.json_output),
                    param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
                    param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
                    #param(name="number_of_tests", value=str(len(self.test_list))),
                    node(name="player", pkg="rosbag", type="play", required="true", output="log", args="--delay=5.0 --clock " +
                                                                                         "--rate=" + str(self.speed_factor_analysis) + " " +
                                                                                         os.path.join(self.generation_config["bagfile_output"], subtest_name +
                                                                                         ".bag")),
                    test({'test-name': "analysing_" + subtest_name, 'pkg': "atf_core", 'type': "analyser.py",
                          'time-limit': str(self.generation_config["time_limit_analysing"]), 'required': "true"})
                )

                xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
                filepath = os.path.join(self.test_generated_analysing_path, "analysing_" + subtest_name) + ".test"
                with open(filepath, "w") as f:
                    f.write(xmlstr)

            # Merging
            test_merge = launch(
                param(name=self.ns + "test_name", value=test_name),
                param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml")),
                param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
                param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
                test({'test-name': "merging", 'pkg': "atf_core", 'type': "merger.py",
                      'time-limit': "10"})
            )
            xmlstr = minidom.parseString(ElementTree.tostring(test_merge)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "merging.test")
            with open(filepath, "w") as f:
                f.write(xmlstr)

            # Uploading
            test_upload = launch()
            if self.upload_data:
                test_upload.append(
                    test({'test-name': "uploading_data", 'pkg': "atf_core", 'type': "test_dropbox_uploader.py",
                          'time-limit': str(self.time_limit_uploading), 'args': "-f " + os.path.join(self.package_src_path, "config/.dropbox_uploader_config") + " upload " + self.bagfile_output + " " + os.path.join(self.package_name, "data")}))

            if self.upload_result:
                test_upload.append(
                    test({'test-name': "uploading_results", 'pkg': "atf_core", 'type': "test_dropbox_uploader.py",
                          'time-limit': str(self.generation_config["time_limit_uploading"]), 'args': "-f " + os.path.join(self.package_src_path, "config/.dropbox_uploader_config") + " upload " + self.generation_config["json_output"] + " " + os.path.join(self.package_name, "results")}))

            xmlstr = minidom.parseString(ElementTree.tostring(test_upload)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "uploading.test")
            with open(filepath, "w") as f:
                f.write(xmlstr)

        print "-- " + self.print_output

    def generate_test_list(self):
        test_list = {}
        suites = self.load_yaml(os.path.join(self.package_src_path, self.generation_config["suites_file"]))
        #print "suites=", suites

        suite_nr = 0
        for suite_name, suite_data in suites.items():
            #print "\nsuite_name=", suite_name
            #print "suite_nr=", suite_nr
            #print "suite_data=", suite_data

            # check that at least one combination of test_config, robot and robot_env is specified
            variation_elements = ["test_configs", "robots", "robot_envs"]
            variation_elements_singular = ["test_config", "robot", "robot_env"]
            variation_abbreviations = ["c", "r", "e"]
            for variation in variation_elements:
                try:
                    if len(suite_data[variation]) < 1:
                        print "-- ATF error: an empty list for '%s' is specified in test suite '%s', defined in test suites file '%s'." % (variation, suite_name, os.path.join(self.package_src_path, self.generation_config["suites_file"]))
                        #variation_elements.remove(variation)
                        sys.exit(1)
                except KeyError as e:
                    print "-- ATF error: no %s specfied in test suite '%s', defined in test suites file '%s'" % (e, suite_name, os.path.join(self.package_src_path, self.generation_config["suites_file"]))
                    sys.exit(1)

            variations = [dict(zip(variation_elements, prod)) for prod in it.product(*(suite_data[varName] for varName in variation_elements))]
            #print "variations", variations
            
            for test in variations:
                #print "test", test
                test_name = "ts" + str(suite_nr)
                for name, value in test.items():
                    test_name += "_" + variation_abbreviations[variation_elements.index(name)] + str(suite_data[name].index(value))
                test_list[test_name] = {}
                
                # fix keys to be sinular not plural any more
                for key in variation_elements:
                    new_key = variation_elements_singular[variation_elements.index(key)]
                    test[new_key] = test.pop(key)

                test_list[test_name] = test
                test_list[test_name]["subtests"] = []
                for repetition_nr in range(0, self.generation_config["repetitions"]):
                    test_name_rep = test_name + "_" + str(repetition_nr)
                    #print "test_name_rep=", test_name_rep
                    test_list[test_name]["subtests"].append(test_name_rep)
            suite_nr += 1

        if self.generation_config["yaml_output"] != "":
            stream = file(self.test_generated_path + "/test_list.yaml", 'w')
            yaml.dump(deepcopy(self.list_to_array(test_list)), stream, default_flow_style=False)

        if self.generation_config["json_output"] != "":
            stream = file(self.test_generated_path + "/test_list.json", 'w')
            json.dump(deepcopy(self.list_to_array(test_list)), stream)
        else:
            error_message = "Error: Output directory for .json files must be specified!"
            print error_message
            self.print_output = "ATF: Test generation failed! " + error_message
        #print "test_list=", test_list
        return test_list

    def list_to_array(self, org_list):
        temp_list = self.natural_sort(copy(org_list))
        output_array = []

        for item in temp_list:
            output_array.append({item: org_list[item]})

        return output_array

    def natural_sort(self, l):
        return sorted(l, key=self.natural_sort_key)

    @staticmethod
    def natural_sort_key(s, _nsre=re.compile('([0-9]+)')):
        return [int(text) if text.isdigit() else text.lower() for text in re.split(_nsre, s)]

    def load_yaml(self, filename):
        try:
            with open(filename, 'r') as stream:
                return yaml.load(stream)
        except IOError:
            if filename != "":
                error_message = "Error: File '" + filename + "' not found!"
                print error_message
            self.print_output = "ATF: Test generation failed! " + error_message
            sys.exit(1)
            return {}

    @staticmethod
    def remove_pkgname(text, pkgname):
        return text[len(pkgname):]

    def get_path(self, path):
        try:
            rospkg.RosPack().get_path(path.split("/")[0]) + self.remove_pkgname(path, path.split("/")[0])
        except rospkg.common.ResourceNotFound:
            return path
        else:
            return rospkg.RosPack().get_path(path.split("/")[0]) + self.remove_pkgname(path, path.split("/")[0])


if __name__ == '__main__':
    GenerateTests(sys.argv).generate_tests()
