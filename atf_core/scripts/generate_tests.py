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

from xml.etree import ElementTree
from xml.dom import minidom

from copy import deepcopy, copy


class GenerateTests:
    def __init__(self):
        generation_config = self.load_yaml(rospkg.RosPack().get_path("atf_core") +
                                           "/config/test_generation_config.yaml")

        self.test_suite_file = rospkg.RosPack().get_path(generation_config["test_suite_file"].split("/")[0]) + self.\
            remove_pkgname(generation_config["test_suite_file"], generation_config["test_suite_file"].split("/")[0])

        self.test_config_file = rospkg.RosPack().get_path(generation_config["test_config_file"].split("/")[0]) + self.\
            remove_pkgname(generation_config["test_config_file"], generation_config["test_config_file"].split("/")[0])

        self.bagfile_output = rospkg.RosPack().get_path(generation_config["bagfile_output"].split("/")[0]) + self.\
            remove_pkgname(generation_config["bagfile_output"], generation_config["bagfile_output"].split("/")[0])

        self.robot_config_path = rospkg.RosPack().get_path(generation_config["robot_config_path"].split("/")[0]) +\
            self.remove_pkgname(generation_config["robot_config_path"],
                                generation_config["robot_config_path"].split("/")[0])

        self.test_application_path = rospkg.RosPack().get_path(generation_config["test_application_path"].
                                                               split("/")[0]) +\
            self.remove_pkgname(generation_config["test_application_path"],
                                generation_config["test_application_path"].split("/")[0])

        self.move_group_launch = rospkg.RosPack().get_path(generation_config["move_group_launch"].split("/")[0]) +\
            self.remove_pkgname(generation_config["move_group_launch"],
                                generation_config["move_group_launch"].split("/")[0])

        if generation_config["result_yaml_output"] != "":
            self.yaml_output = rospkg.RosPack().get_path(generation_config["result_yaml_output"].split("/")[0]) + self.\
                remove_pkgname(generation_config["result_yaml_output"],
                               generation_config["result_yaml_output"].split("/")[0])
        else:
            self.yaml_output = generation_config["result_yaml_output"]

        self.json_output = rospkg.RosPack().get_path(generation_config["result_json_output"].split("/")[0]) + self.\
            remove_pkgname(generation_config["result_json_output"],
                           generation_config["result_json_output"].split("/")[0])

        self.time_limit = generation_config["time_limit"]

        self.test_repetitions = generation_config["test_repetitions"]

        self.test_list = {}

        # Empty folders
        if os.path.exists(rospkg.RosPack().get_path("atf_core") + "/test/generated/recording/"):
            shutil.rmtree(rospkg.RosPack().get_path("atf_core") + "/test/generated/recording/")
        os.makedirs(rospkg.RosPack().get_path("atf_core") + "/test/generated/recording/")

        if os.path.exists(rospkg.RosPack().get_path("atf_core") + "/test/generated/analysing/"):
            shutil.rmtree(rospkg.RosPack().get_path("atf_core") + "/test/generated/analysing/")
        os.makedirs(rospkg.RosPack().get_path("atf_core") + "/test/generated/analysing/")

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

        for idx, item in enumerate(self.test_list):
            # Create .test file
            em = lxml.builder.ElementMaker()
            launch = em.launch
            arg = em.arg
            include = em.include
            node = em.node
            param = em.param

            robot_config = self.load_yaml(self.robot_config_path + self.test_list[item]["robot"] + "/robot_config.yaml")
            robot_bringup_launch = robot_config["robot_bringup_launch"]
            robot_bringup = rospkg.RosPack().get_path(robot_bringup_launch.split("/")[0]) +\
                self.remove_pkgname(robot_bringup_launch, robot_bringup_launch.split("/")[0])

            # Recording
            test_record = launch(
                param(name="use_sim_time", value="true"),
                param(name="test_name", value=item),
                param(name="test_config", value=self.test_list[item]["test_config"]),
                param(name="scene_config", value=self.test_list[item]["scene_config"]),
                param(name="planer_id", value=self.test_list[item]["planer_id"]),
                param(name="eef_step", value=str(self.test_list[item]["eef_step"])),
                param(name="jump_threshold", value=str(self.test_list[item]["jump_threshold"])),
                param(name="planning_method", value=self.test_list[item]["planning_method"]),
                param(name="recorder/test_config_file", value=self.test_config_file),
                param(name="recorder/bagfile_output", value=self.bagfile_output),
                arg(name="robot", value=self.test_list[item]["robot"]),
                arg(name="rc_path", value=self.robot_config_path),
                include(arg(name="gui", value="false"), file=robot_bringup),
                include(file=self.move_group_launch),
                node(param(name="robot_config_file", value="$(arg rc_path)$(arg robot)/robot_config.yaml"),
                     name="atf_recorder", pkg="atf_recorder", type="recorder_core.py", output="screen"),
                node(name="obstacle_distance_node", pkg="atf_recorder_plugins", type="obstacle_distance_node",
                     output="screen"),
                include(file=self.test_application_path)
            )

            xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
            with open(rospkg.RosPack().get_path("atf_core") + "/test/generated/recording/" + item + ".test", "w") as f:
                f.write(xmlstr)

            # Analysing
            em = lxml.builder.ElementMaker()
            launch = em.launch
            test = em.test
            node = em.node
            param = em.param

            test_analyse = launch(
                param(name="use_sim_time", value="true"),
                param(name="analysing/test_name", value=item),
                param(name="analysing/test_config", value=self.test_list[item]["test_config"]),
                param(name="analysing/test_config_file", value=self.test_config_file),
                param(name="analysing/result_yaml_output", value=self.yaml_output),
                param(name="analysing/result_json_output", value=self.json_output),
                test({'test-name': "test_analysing", 'pkg': "atf_core", 'type': "test_builder.py",
                      'time-limit': str(self.time_limit)}),
                node(name="player", pkg="rosbag", type="play", output="screen", args="--delay=5.0 --clock " +
                                                                                     self.bagfile_output + item +
                                                                                     ".bag")
            )

            xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
            with open(rospkg.RosPack().get_path("atf_core") + "/test/generated/analysing/" + item + ".test", "w") as f:
                f.write(xmlstr)

        print "Generation done!"

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
            yaml.dump(deepcopy(self.list_to_array(test_list_org)), stream)

        stream = file(self.json_output + "/test_list.json", 'w')
        json.dump(self.list_to_array(test_list_org), stream)

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

    @staticmethod
    def load_yaml(filename):
        with open(filename, 'r') as stream:
            return yaml.load(stream)

    @staticmethod
    def remove_pkgname(text, pkgname):
        return text[len(pkgname):]


if __name__ == '__main__':
    GenerateTests().generate_tests()
