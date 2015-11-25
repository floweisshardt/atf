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

        self.print_output = "Generation done!"
        self.arguments = arguments
        generation_config = self.load_yaml(self.arguments[1])
        try:
            self.test_suite_file = self.get_path(generation_config["test_suite_file"])
            self.test_config_file = self.get_path(generation_config["test_config_file"])
            self.bagfile_output = self.get_path(generation_config["bagfile_output"])
            self.robot_config_path = self.get_path(generation_config["robot_config_path"])
            self.test_application_path = self.get_path(generation_config["test_application_path"])
            self.additional_launch_file = self.get_path(generation_config["additional_launch_file"])

            if generation_config["result_yaml_output"] != "":
                self.yaml_output = self.get_path(generation_config["result_yaml_output"])
            else:
                self.yaml_output = generation_config["result_yaml_output"]

            self.json_output = self.get_path(generation_config["result_json_output"])
            self.time_limit_recording = generation_config["time_limit_recording"]
            self.time_limit_analysing = generation_config["time_limit_analysing"]
            self.test_repetitions = generation_config["test_repetitions"]
        except KeyError:
            self.test_suite_file = ""
            self.test_config_file = ""
            self.bagfile_output = ""
            self.robot_config_path = ""
            self.test_application_path = ""
            self.additional_launch_file = ""
            self.yaml_output = ""
            self.json_output = ""
            self.time_limit_recording = 0
            self.time_limit_analysing = 0
            self.test_repetitions = 1

        self.test_list = {}

        # Empty folders
        if os.path.exists(self.arguments[2]):
            shutil.rmtree(self.arguments[2])
        os.makedirs(self.arguments[2] + "recording/")
        os.makedirs(self.arguments[2] + "analysing/")

        if os.path.exists(self.bagfile_output):
            shutil.rmtree(self.bagfile_output)
        os.makedirs(self.bagfile_output)

        if os.path.exists(self.json_output):
            shutil.rmtree(self.json_output)
        os.makedirs(self.json_output)

        if self.yaml_output != "":
            if os.path.exists(self.yaml_output):
                shutil.rmtree(self.yaml_output)
            os.makedirs(self.yaml_output)

        self.generate_test_list()

    def generate_tests(self):

        for item in self.test_list:
            # Create .test file
            em = lxml.builder.ElementMaker()
            launch = em.launch
            arg = em.arg
            include = em.include
            node = em.node
            param = em.param
            rosparam = em.rosparam

            robot_config = self.load_yaml(self.robot_config_path + self.test_list[item]["robot"] + "/robot_config.yaml")

            # Recording
            test_record = launch(
                include(arg(name="test_status_list", value=self.arguments[2] + "test_status.yaml"),
                        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                param(name="use_sim_time", value="true"),
                param(name="test_name", value=item),
                param(name="test_config", value=self.test_list[item]["test_config"]),
                param(name="scene_config", value=self.test_list[item]["scene_config"]),
                param(name="robot_config", value=self.robot_config_path + self.test_list[item]["robot"] +
                      "/robot_config.yaml"),
                param(name="number_of_tests", value=str(len(self.test_list)))
            )

            for config_param in self.test_list[item]:
                if config_param == "test_config" or config_param == "scene_config" or config_param == "robot":
                    continue
                test_record.append(param(name=config_param, value=str(self.test_list[item][config_param])))

            test_record.append(arg(name="robot", value=self.test_list[item]["robot"]))
            test_record.append(arg(name="rc_path", value=self.robot_config_path))

            if robot_config["robot_bringup_launch"] != "":
                test_record.append(include(file=self.get_path(robot_config["robot_bringup_launch"])))

            if self.additional_launch_file != "":
                test_record.append(include(file=self.additional_launch_file))

            test_record.append(node(param(name="/test_config_file", value=self.test_config_file),
                                    param(name="/bagfile_output", value=self.bagfile_output),
                                    name="atf_recorder", pkg="atf_recorder", type="recorder_core.py", output="screen"))
            test_record.append(rosparam(ns="obstacle_distance_node", command="load",
                                        file=self.robot_config_path + self.test_list[item]["robot"] +
                                        "/robot_config.yaml"))
            test_record.append(node(name="obstacle_distance_node", pkg="obstacle_distance",
                                    type="obstacle_distance_node", output="screen"))
            test_record.append(include(arg(name="time_limit", value=str(self.time_limit_recording)),
                                       file=self.test_application_path))

            for params in robot_config["additional_parameter"]:
                test_record.append(param(name=str(params["name"]), value=str(params["value"])))

            for args in robot_config["additional_arguments"]:
                test_record.append(arg(name=str(args["name"]), value=str(args["value"])))

            xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
            with open(self.arguments[2] + "recording/" + item + ".test", "w") as f:
                f.write(xmlstr)

            # Analysing
            em = lxml.builder.ElementMaker()
            launch = em.launch
            test = em.test
            node = em.node
            param = em.param

            test_analyse = launch(
                include(arg(name="test_status_list", value=self.arguments[2] + "test_status.yaml"),
                        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                param(name="use_sim_time", value="true"),
                param(name="analysing/test_name", value=item),
                param(name="analysing/test_config", value=self.test_list[item]["test_config"]),
                param(name="analysing/test_config_file", value=self.test_config_file),
                param(name="analysing/result_yaml_output", value=self.yaml_output),
                param(name="analysing/result_json_output", value=self.json_output),
                param(name="number_of_tests", value=str(len(self.test_list))),
                test({'test-name': "test_analysing", 'pkg': "atf_core", 'type': "test_builder.py",
                      'time-limit': str(self.time_limit_analysing)}),
                node(name="player", pkg="rosbag", type="play", output="screen", args="--delay=5.0 --clock " +
                                                                                     self.bagfile_output + item +
                                                                                     ".bag")
            )

            xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
            with open(self.arguments[2] + "analysing/" + item + ".test", "w") as f:
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
            stream = file(self.yaml_output + "/test_list.yaml", 'w')
            yaml.dump(deepcopy(self.list_to_array(test_list_org)), stream, default_flow_style=False)

        if self.json_output != "":
            stream = file(self.json_output + "/test_list.json", 'w')
            json.dump(self.list_to_array(test_list_org), stream)
        else:
            print "Error: Output directory for .json files must be specified!"
            self.print_output = "Generation failed!"

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
                print "Error: File '" + filename + "' not found!"
            self.print_output = "Generation failed!"
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
