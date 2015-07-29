#!/usr/bin/env python
import rospy
import yaml
import re
import itertools as it
import psutil
import rospkg
import rosparam
import thread
import os

from copy import copy


class AutomatedTesting:
    def __init__(self):

        self.test_suites = rospkg.RosPack().get_path("atf_testing") + "/config/test_suite.yaml"
        self.test_list = {}

    def start(self):

        self.generate_test_list()

        for test in self.test_list:

            # Parse rosparams
            rosparam.set_param("/test_name", test)
            rosparam.set_param("/test_config", self.test_list[test]["test_config"])
            rosparam.set_param("/scene_config", self.test_list[test]["scene_config"])
            rosparam.set_param("/planer_id", self.test_list[test]["planer_id"])
            rosparam.set_param("/eef_step", str(self.test_list[test]["eef_step"]))
            rosparam.set_param("/jump_threshold", str(self.test_list[test]["jump_threshold"]))
            rosparam.set_param("/planning_method", self.test_list[test]["planning_method"])
            rosparam.set_param("/recorder_finished", "False")

            # Start roslaunch
            thread.start_new_thread(os.system,("roslaunch atf_testing automated_testing.launch",))
            # os.system("roslaunch atf_testing automated_testing.launch")

            while not rosparam.get_param("recorder_finished"):
                pass
            pid = self.get_pid("roslaunch")
            os.system("kill -15 " + str(pid))
            try:
                pid = self.get_pid("gazebo")
            except IndexError:
                pass
            else:
                os.system("kill -15 " + str(pid))
            try:
                pid = self.get_pid("move_group")
            except IndexError:
                pass
            else:
                os.system("kill -15 " + str(pid))

    def generate_test_list(self):
        temp_config = {}

        with open(self.test_suites, 'r') as stream:
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

        stream = file(rospkg.RosPack().get_path("atf_testing") + "/config/test_list.yaml", 'w')
        yaml.dump(self.list_to_array(), stream)

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

    @staticmethod
    def get_pid(name):
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]


if __name__ == "__main__":
    rospy.init_node("automated_testing")
    at = AutomatedTesting()
    at.start()
