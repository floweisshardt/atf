#!/usr/bin/env python
import rospy
import unittest
import rostest
import shutil
import os
import sys

from atf_core import ATFConfigurationParser


class Cleaner():
    def __init__(self, package_name):
        self.result = False

        self.atf_configuration_parser = ATFConfigurationParser(package_name)

    def clean(self):
        if os.path.exists(self.atf_configuration_parser.generation_config["bagfile_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["bagfile_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["txt_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["txt_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["json_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["json_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["yaml_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["yaml_output"])
        self.result = True

class TestCleaning(unittest.TestCase):
    def test_cleaning_results(self):
        cleaner = Cleaner(sys.argv[1])
        cleaner.clean()
        self.assertTrue(cleaner.result, "Could not clean results.")

if __name__ == '__main__':
    print "cleaning for package", sys.argv[1]
    if "standalone" in sys.argv:
        cleaner = Cleaner(sys.argv[1])
    else:
        rostest.rosrun("atf_core", 'cleaning', TestCleaning, sysargs=sys.argv)
