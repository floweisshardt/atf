#!/usr/bin/env python
import argparse
import unittest
import rostest
import shutil
import os
import sys

from atf_core.configuration_parser import ATFConfigurationParser


class Cleaner():
    def __init__(self, package_name, test_generation_config_file):
        # parse configuration
        self.atf_configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)

    def clean(self, dry_run):
        if dry_run:
            return True

        if os.path.exists(self.atf_configuration_parser.generation_config["bagfile_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["bagfile_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["txt_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["txt_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["json_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["json_output"])
        if os.path.exists(self.atf_configuration_parser.generation_config["yaml_output"]):
            shutil.rmtree(self.atf_configuration_parser.generation_config["yaml_output"])
        return True

class TestCleaning(unittest.TestCase):

    def test_cleaning_results(self):
        cleaner = Cleaner(args.pkg, args.test_generation_config_file)
        self.assertTrue(cleaner.clean(args.dry_run), "Could not clean results.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Manual exection of ATF cleaning phase.', formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('pkg', type=str,
                        help='test package name')
    parser.add_argument('-g', dest='test_generation_config_file',
                        default='atf/test_generation_config.yaml',
                        help='path to test_generation_config file, relative to package root')
    parser.add_argument('-e', dest='execute_as_test', action='count',
                        help='execute as rostest')
    parser.add_argument('-d', dest='dry_run', action='count',
                        help='execute dry run')

    args, unknown = parser.parse_known_args()
    if args.execute_as_test:
        rostest.rosrun("atf_core", 'cleaning', TestCleaning)
    else:
        args = parser.parse_args() # strictly parse only known arguments again. will raise an error if unknown arguments are specified
        print("cleaning for package '%s' and test generation config file '%s'" %(args.pkg, args.test_generation_config_file))
        cleaner = Cleaner(args.pkg, args.test_generation_config_file)
        cleaner.clean(args.dry_run)
