#!/usr/bin/env python

from atf_core import Testblock

class Test:
    def __init__(self):
        self.name = None
        self.testsuite = None
        self.testsuite_name = None
        self.test_config = None
        self.test_config_name = None
        self.robot = None
        self.robot_name = None
        self.robot_env = None
        self.robot_env_name = None
        self.generation_config = None
        # testblocks with metrics
        testblocks = []
        
        # result data
        self.result = None
        self.groundtruth_result = None
        self.groundtruth_error_message = None
        
    
    def print_to_terminal(self):
        print "\n----------------------------- " + self.name + " ----------------------------------"
        print "testsuite_name:", self.testsuite_name
        print "test_config_name:", self.test_config_name
        print "robot_name:", self.robot_name
        print "robot_env_name:", self.robot_env_name
        print "---data---"
        print "testsuite:", self.testsuite
        print "test_config:", self.test_config
        print "robot:", self.robot
        print "robot_env:", self.robot_env
        print "generation_config:", self.generation_config


    def print_result_to_terminal(self):
        print "\n----------------------------- " + self.name + " ----------------------------------"
        print "result:", self.result
        print "groundtruth_result: ", self.groundtruth_result
        print "groundtruth_error_message", self.groundtruth_error_message
