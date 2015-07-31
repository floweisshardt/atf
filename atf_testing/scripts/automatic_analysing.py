#!/usr/bin/env python
import rospy
import os
import rosparam
import yaml
import rosgraph


class AutomaticAnalysing:
    def __init__(self):
        bag_folder = rosparam.get_param(rospy.get_name() + "/bagfile_input")
        self.bagfile_list = []
        self.test_names = []
        self.test_configs = {}
        self.test_list_file = rosparam.get_param(rospy.get_name() + "/test_list_file")
        self.test_config_file = rosparam.get_param(rospy.get_name() + "/test_config_file")
        try:
            self.yaml_output = rosparam.get_param(rospy.get_name() + "/result_yaml_output")
        except rosgraph.masterapi.MasterError:
            self.yaml_output = False
            pass

        self.json_output = rosparam.get_param(rospy.get_name() + "/result_json_output")

        with open(self.test_list_file, 'r') as stream:
            self.test_list = yaml.load(stream)

        for bagfile in os.listdir(bag_folder):
            if bagfile.endswith(".bag"):
                self.bagfile_list.append(bag_folder + bagfile)
                self.test_names.append(bagfile.split(".")[0])

        for item in self.test_list:
            if item.keys()[0] in self.test_names:
                self.test_configs.update(item)

    def start(self):

        for idx, bagfile in enumerate(self.bagfile_list):

            rospy.loginfo("---- Start test '" + self.test_names[idx] + "' ----")

            # Parse rosparams
            rosparam.set_param("/analysing/test_config", self.test_configs[self.test_names[idx]]["test_config"])
            rosparam.set_param("/analysing/test_name", self.test_names[idx])
            rosparam.set_param("/analysing/test_config_file", self.test_config_file)
            if self.yaml_output is not False:
                rosparam.set_param("/analysing/result_yaml_output", self.yaml_output)
            rosparam.set_param("/analysing/result_json_output", self.json_output)

            # Start roslaunch
            os.system("roslaunch atf_testing start_analysing.launch bagfile:=" + bagfile)

            rospy.loginfo("---- Finished test '" + self.test_names[idx] + "' ----")

if __name__ == "__main__":
    rospy.init_node("automatic_analysing")
    aa = AutomaticAnalysing()
    aa.start()
