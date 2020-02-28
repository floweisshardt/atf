#!/usr/bin/env python
import rospy
import unittest
import rostest
import shutil
import os
import sys

from atf_core import ATFConfigurationParser


class Cleaner():
    def __init__(self, arguments):
        self.result = False

        package_name = arguments[1]
        test_generation_config_file = arguments[2]
        self.atf_configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)

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
        cleaner = Cleaner(sys.argv)
        cleaner.clean()
        self.assertTrue(cleaner.result, "Could not clean results.")

if __name__ == '__main__':
    print "cleaning for package", sys.argv[1]
    if "standalone" in sys.argv:
        cleaner = Cleaner(sys.argv)
    else:
        rostest.rosrun("atf_core", 'cleaning', TestCleaning, sysargs=sys.argv)
