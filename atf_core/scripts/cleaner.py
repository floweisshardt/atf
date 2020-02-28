#!/usr/bin/env python
import rospy
import unittest
import rostest
import shutil
import os
import sys

from atf_core import ATFConfigurationParser


class Cleaner():
    def __init__(self, package_name, test_generation_config_file):
        self.result = False

        # parse configuration
        self.atf_configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)

    def clean(self):
        print self.atf_configuration_parser.generation_config["txt_output"]
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
        cleaner = Cleaner(sys.argv[1], sys.argv[2])
        cleaner.clean()
        self.assertTrue(cleaner.result, "Could not clean results.")

if __name__ == '__main__':
    if len(sys.argv) == 2:
        package_name = sys.argv[1]
        test_generation_config_file = "atf/test_generation_config.yaml"
    elif len(sys.argv) > 2:
        package_name = sys.argv[1]
        test_generation_config_file = sys.argv[2]
    else:
        print "ERROR: please specify a test package"
        print "usage: rosrun atf_core cleaner.py <<ATF TEST PACKAGE>> [<<TEST_GENERATION_CONFIG_FILE>>]"
        sys.exit(1)
    print "cleaning for package '%s' and test generation config file '%s'" %(package_name, test_generation_config_file)

    if "execute_as_test" in sys.argv:
        rostest.rosrun("atf_core", 'cleaning', TestCleaning, sysargs=package_name + " " + test_generation_config_file)
    else:
        cleaner = Cleaner(package_name, test_generation_config_file)
        cleaner.clean()
