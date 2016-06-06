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
        self.package_path = arguments[2]
        self.generation_config = self.load_yaml(self.package_path + "/config/test_generation_config.yaml")

        # required parameters
        try:
            self.test_suite_file = os.path.join(self.package_path, self.generation_config["test_suite_file"])
            self.bagfile_output = os.path.join(self.package_path, self.generation_config["bagfile_output"])

            if self.generation_config["yaml_output"] != "":
                self.yaml_output = os.path.join(self.package_path, self.generation_config["yaml_output"])
            else:
                self.yaml_output = self.generation_config["yaml_output"]

            self.json_output = os.path.join(self.package_path, self.generation_config["json_output"])
            self.time_limit_recording = self.generation_config["time_limit_recording"]
            self.time_limit_analysing = self.generation_config["time_limit_analysing"]
            self.time_limit_uploading = self.generation_config["time_limit_uploading"]
            self.test_repetitions = self.generation_config["test_repetitions"]
        except KeyError as e:
            error_message = "Error: parsing test configuration failed. Missing Key: " + str(e)
            print error_message
            self.print_output = "ATF: Test generation failed! " + error_message
            sys.exit(1)

        # optional parameters
        try:
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

        self.test_generated_path = os.path.join(self.package_path, "test_generated")
        self.test_generated_recording_path = os.path.join(self.test_generated_path, "recording")
        self.test_generated_analysing_path = os.path.join(self.test_generated_path, "analysing")
        self.create_folders()
        self.generate_test_list()

    def create_folders(self):
        # delete of test_generated directory and create new one
        if os.path.exists(self.test_generated_recording_path):
            shutil.rmtree(self.test_generated_path)
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

        for test_name in self.test_list:
            robot_config = self.load_yaml(os.path.join(self.package_path, self.generation_config["robot_config_path"], self.test_list[test_name]["robot"], "robot_config.yaml"))
            
            #print "self.test_list[test_name]=", self.test_list[test_name]

            # Cleaning
            test_clean = launch(
                param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                param(name=self.ns + "bag_output", value=self.bagfile_output),
                param(name=self.ns + "yaml_output", value=self.yaml_output),
                param(name=self.ns + "json_output", value=self.json_output),
                test({'test-name': "cleaning", 'pkg': "atf_core", 'type': "cleaner.py",
                      'time-limit': "10"})
            )
            xmlstr = minidom.parseString(ElementTree.tostring(test_clean)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "cleaning.test")
            with open(filepath, "w") as f:
                f.write(xmlstr)

            # Recording
            test_record = launch(
                #arg(name="robot", value=self.test_list[test_name]["robot"]),
                #include(arg(name="test_status_list", value="$(find " + self.package_name + ")/test_status.yaml"),
                #        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                param(name=self.ns + "test_name", value=test_name),
                param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                param(name=self.ns + "scene_config_name", value=self.test_list[test_name]["scene_config"]),
                #rosparam(param=self.ns + "scene_config", command="load", file="$(find " + self.package_name + ")/" + self.generation_config["scene_config_file"]),
                param(name=self.ns + "robot_config_name", value=self.test_list[test_name]["robot"]),
                rosparam(param=self.ns + "robot_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["robot_config_path"], self.test_list[test_name]["robot"], "robot_config.yaml")),
                param(name=self.ns + "bagfile_output", value=self.bagfile_output),
                param(name=self.ns + "json_output", value=self.json_output),
                param(name=self.ns + "yaml_output", value=self.yaml_output),
                #param(name=self.ns + "number_of_tests", value=str(len(self.test_list)))
            )

            for config_param in self.test_list[test_name]:
                if config_param == "test_config" or config_param == "scene_config" or config_param == "robot":
                    continue
                test_record.append(param(name=config_param, value=str(self.test_list[test_name][config_param])))

            if robot_config["robot_bringup_launch"] != "":
                test_record.append(include(file="$(find " + self.package_name + ")/" + robot_config["robot_bringup_launch"]))

            if self.generation_config["additional_launch_file"] != "":
                test_record.append(include(file="$(find " + self.package_name + ")/" + self.generation_config["additional_launch_file"]))

            #test_record.append(node(param(name="/test_config_file", value="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
            #                        param(name="/bagfile_output", value=self.bagfile_output),
            #                        name="atf_recorder", pkg="atf_recorder", type="recorder_core.py", output="screen"))

            for params in robot_config["additional_parameter"]:
                test_record.append(param(name=str(params["name"]), value=str(params["value"])))

            for args in robot_config["additional_arguments"]:
                test_record.append(arg(name=str(args["name"]), value=str(args["value"])))

            test_record.append(test({'test-name': "recording_" + test_name, 'pkg': self.package_name, 'type': self.generation_config['app_executable'],
                      'time-limit': str(self.time_limit_recording)}))

            xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_recording_path, "recording_" + test_name) + ".test"
            with open(filepath, "w") as f:
                f.write(xmlstr)

            # Analysing
            test_analyse = launch(
                #include(arg(name="test_status_list", value="$(find " + self.package_name + ")/test_status.yaml"),
                #        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                param(name=self.ns + "test_name", value=test_name),
                param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                #param(name="test_config", value=self.test_list[test_name]["test_config"]),
                #param(name="test_config_file", value="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                param(name=self.ns + "test_generated_path", value="$(find " + self.package_name + ")/test_generated"),
                #param(name="yaml_output", value=self.yaml_output),
                #param(name="json_output", value=self.json_output),
                param(name=self.ns + "json_output", value=self.json_output),
                param(name=self.ns + "yaml_output", value=self.yaml_output),
                #param(name="number_of_tests", value=str(len(self.test_list))),
                node(name="player", pkg="rosbag", type="play", output="log", args="--delay=5.0 --clock " +
                                                                                     "--rate=" + str(self.speed_factor_analysis) + " " +
                                                                                     self.bagfile_output + test_name +
                                                                                     ".bag"),
                test({'test-name': "analysing_" + test_name, 'pkg': "atf_core", 'type': "analyser.py",
                      'time-limit': str(self.time_limit_analysing)})
            )

            xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_analysing_path, "analysing_" + test_name) + ".test"
            with open(filepath, "w") as f:
                f.write(xmlstr)

            # Merging
            test_merge = launch(
                param(name=self.ns + "test_name", value=test_name),
                param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + self.generation_config["test_config_file"]),
                param(name=self.ns + "yaml_output", value=self.yaml_output),
                param(name=self.ns + "json_output", value=self.json_output),
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
                          'time-limit': str(self.time_limit_uploading), 'args': "-f " + os.path.join(self.package_path, "config/.dropbox_uploader_config") + " upload " + self.bagfile_output + " " + os.path.join(self.package_name, "data")}))

            if self.upload_result:
                test_upload.append(
                    test({'test-name': "uploading_results", 'pkg': "atf_core", 'type': "test_dropbox_uploader.py",
                          'time-limit': str(self.time_limit_uploading), 'args': "-f " + os.path.join(self.package_path, "config/.dropbox_uploader_config") + " upload " + self.json_output + " " + os.path.join(self.package_name, "results")}))

            xmlstr = minidom.parseString(ElementTree.tostring(test_upload)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "uploading.test")
            with open(filepath, "w") as f:
                f.write(xmlstr)

        print "-- " + self.print_output

    def generate_test_list(self):
        test_list_org = {}

        test_data = self.load_yaml(self.test_suite_file)
        for suite in test_data:

            suite_data = copy(test_data[suite])
            items = sorted(suite_data)
            temp = [dict(zip(items, prod)) for prod in it.product(*(suite_data[varName] for varName in items))]

            for i in xrange(0, len(temp)):
                test_name_org = suite[0] + suite[4] + suite.split("_")[1] + "_" + "t" + str(i + 1)
                test_list_org[test_name_org] = {}
                test_list_org[test_name_org].update(temp[i])
                test_list_org[test_name_org]["test_repetitions"] = self.test_repetitions
                test_list_org[test_name_org]["subtests"] = []

                for j in xrange(0, self.test_repetitions):
                    test_name = suite[0] + suite[4] + suite.split("_")[1] + "_" + "t" + str(i + 1) + "_" + str(j + 1)
                    test_list_org[test_name_org]["subtests"].append(test_name)
                    self.test_list[test_name] = {}
                    self.test_list[test_name].update(temp[i])

        if self.yaml_output != "":
            stream = file(self.test_generated_path + "/test_list.yaml", 'w')
            yaml.dump(deepcopy(self.list_to_array(test_list_org)), stream, default_flow_style=False)

        if self.json_output != "":
            stream = file(self.test_generated_path + "/test_list.json", 'w')
            json.dump(self.list_to_array(test_list_org), stream)
        else:
            error_message = "Error: Output directory for .json files must be specified!"
            print error_message
            self.print_output = "ATF: Test generation failed! " + error_message

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
