#!/usr/bin/env python
import copy
import json
import os
import shutil
import yaml

from atf_core import Testblock

class Test:
    def __init__(self):
        self.package_name = None
        self.name = None
        self.testsuite = None
        self.testsuite_name = None
        self.test_config = None
        self.test_config_name = None
        self.robot_config = None
        self.robot_name = None
        self.robot_env_config = None
        self.robot_env_name = None
        self.generation_config = None

        # testblocks with metrics
        metrics_handle = None
        testblocks = []
        
        # result data
        self.result = None        
    
    def print_to_terminal(self):
        print "\n----------------------------- " + self.name + " ----------------------------------"
        print "testsuite_name:", self.testsuite_name
        print "test_config_name:", self.test_config_name
        print "robot_name:", self.robot_name
        print "robot_env_name:", self.robot_env_name
        print "---data---"
        print "testsuite:", self.testsuite
        print "test_config:", self.test_config
        print "robot_config:", self.robot_config
        print "robot_env_config:", self.robot_env_config
        print "generation_config:", self.generation_config

    def print_result_to_terminal(self):
        print "\n----------------------------- " + self.name + " ----------------------------------"
        print "result:", self.result