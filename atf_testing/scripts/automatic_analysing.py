#!/usr/bin/env python
import rospy
import rospkg
import os
import rosparam
import yaml


class AutomaticAnalysing:
    def __init__(self):
        bag_folder = rospkg.RosPack().get_path("atf_recorder") + "/data/"
        self.bagfile_list = []
        self.test_names = []
        self.test_configs = {}

        with open(rospkg.RosPack().get_path("atf_testing") + "/config/test_list.yaml", 'r') as stream:
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

            # Parse rosparams
            os.environ['BAGFILE'] = bagfile
            rosparam.set_param("/test_config", self.test_configs[self.test_names[idx]]["test_config"])
            rosparam.set_param("/test_name", self.test_names[idx])

            # Start roslaunch
            os.system("roslaunch atf_testing test_analyse.launch")

if __name__ == "__main__":
    rospy.init_node("automatic_analysing")
    aa = AutomaticAnalysing()
    aa.start()
