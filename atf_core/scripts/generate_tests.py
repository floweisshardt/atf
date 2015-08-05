#!/usr/bin/env python
import rospy
import unittest
import rostest
import yaml
import json
import re
import itertools as it
import rospkg
import rosparam
import os
import lxml.etree
import lxml.builder

from copy import deepcopy, copy


class GenerateTests(unittest.TestCase):

    def setUp(self):
        self.test_suite_file = rosparam.get_param("/test_suite_file")
        self.test_config_file = rosparam.get_param("/test_config_file")
        self.bagfile_output = rosparam.get_param("/bagfile_output")
        self.robot_config_path = rosparam.get_param("/robot_config_path")
        self.test_list = {}

        if not os.path.exists(rospkg.RosPack().get_path("atf_core") + "/test/generated/"):
            os.makedirs(rospkg.RosPack().get_path("atf_core") + "/test/generated/")

        self.generate_test_list()

    def test_GenerateRecordingTests(self):

        for item in self.test_list:

            # Create .test file
            em = lxml.builder.ElementMaker()
            launch = em.launch
            arg = em.arg
            include = em.include
            test = em.test
            node = em.node
            param = em.param

            the_doc = launch(
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
                include(arg(name="gui", value="false"), file="$(find cob_bringup_sim)/launch/robot.launch"),
                include(file="$(find cob_grasping)/launch/move_group.launch"),
                node(param(name="robot_config_file", value="$(arg rc_path)$(arg robot)/robot_config.yaml"),
                     name="atf_recorder",pkg="atf_recorder", type="recorder.py", output="screen"),
                test(param(name="scene_config_file", value="$(find cob_grasping)/config/scene_config.yaml"),
                     param(name="switch_arm", value="False"),
                     param(name="wait_for_user", value="False"),
                     param(name="joint_trajectory_speed", value="0.3"),
                     param(name="max_error", value="50"),
                     param(name="lift_height", value="0.02"),
                     param(name="approach_distance", value="0.14"),
                     param(name="manipulation_repeats", value="1"),
                     param(name="load_obstacles", value="none"),
                     test-name="test_recording", pkg="atf_core", type="test_recording.py", time-limit="300.0")
            )

            tree = lxml.etree.ElementTree(the_doc)
            tree.write(rospkg.RosPack().get_path("atf_core") + "/test/generated/" + item + ".test")

    def test_GenerateAnalysingTests(self):
        pass

    def generate_test_list(self):
        temp_config = {}

        with open(self.test_suite_file, 'r') as stream:
            test_data = yaml.load(stream)

        for suite in test_data:

            suite_data = copy(test_data[suite])
            temp_config["scene_config"] = suite_data["scene_config"]
            temp_config["test_config"] = suite_data["test_config"]
            temp_config["robot"] = suite_data["robot"]

            suite_data.pop("scene_config", None)
            suite_data.pop("test_config", None)
            suite_data.pop("robot", None)

            items = sorted(suite_data)
            temp = [dict(zip(items, prod)) for prod in it.product(*(suite_data[varName] for varName in items))]

            for i in xrange(0, len(temp)):
                test_name = suite[0] + suite[4] + suite.split("_")[1] + "_" + "t" + str(i+1)
                self.test_list[test_name] = copy(temp_config)
                self.test_list[test_name].update(temp[i])

        if not os.path.exists(rospkg.RosPack().get_path("atf_presenter") + "/data/"):
            os.makedirs(rospkg.RosPack().get_path("atf_presenter") + "/data/")

        stream = file(rospkg.RosPack().get_path("atf_core") + "/config/test_list.yaml", 'w')
        yaml.dump(deepcopy(self.list_to_array()), stream)

        stream = file(rospkg.RosPack().get_path("atf_presenter") + "/data/test_list.json", 'w')
        json.dump(self.list_to_array(), stream)

    def list_to_array(self):
        temp_list = self.natural_sort(copy(self.test_list))
        output_array = []

        for item in temp_list:
            output_array.append({item: self.test_list[item]})

        return output_array

    @staticmethod
    def natural_sort(l):
        convert = lambda text: int(text) if text.isdigit() else text.lower()
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        return sorted(l, key=alphanum_key)

if __name__ == '__main__':
    rospy.init_node('generate_tests')
    rostest.rosrun("atf_core", 'generate_tests', GenerateTests, sysargs=None)