#!/usr/bin/env python
import rospy
import unittest
import rostest
import shutil
import os

from atf_core import ATFConfigurationParser


class Cleaner():
    def __init__(self):
        self.result = False

        self.atf_configuration_parser = ATFConfigurationParser()
        self.config = self.atf_configuration_parser.get_config()

    def clean(self):
        if os.path.exists(self.config["bag_output"]):
            shutil.rmtree(self.config["bag_output"])
        if os.path.exists(self.config["json_output"]):
            shutil.rmtree(self.config["json_output"])
        if os.path.exists(self.config["yaml_output"]):
            shutil.rmtree(self.config["yaml_output"])
        self.result = True

class TestMerging(unittest.TestCase):
    def test_cleaning_results(self):
        cleaner = Cleaner()
        cleaner.clean()
        self.assertTrue(cleaner.result, "Could not merge results.")

if __name__ == '__main__':
    rospy.init_node('test_merging')
    rostest.rosrun("atf_core", 'merging', TestMerging, sysargs=None)
