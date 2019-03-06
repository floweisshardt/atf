#!/usr/bin/env python
import copy
import json
import os
import shutil
import yaml

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
        print "robot:", self.robot
        print "robot_env:", self.robot_env
        print "generation_config:", self.generation_config

    def print_result_to_terminal(self):
        print "\n----------------------------- " + self.name + " ----------------------------------"
        print "result:", self.result

    def export_to_file(self):
        #print "result:", result
    
        # we'll always have json export
        if os.path.exists(self.generation_config["json_output"]):
        #    shutil.rmtree(self.config["json_output"]) #FIXME will fail if multiple test run concurrently
            pass
        else:
            os.makedirs(self.generation_config["json_output"])
        shutil.copyfile(os.path.join("/home/fmw/git/atf/build/atf_test_app_time/test_generated", "test_list.json"), os.path.join(self.generation_config["json_output"], "test_list.json"))
        filename = os.path.join(self.generation_config["json_output"], self.name + ".json")
        stream = file(filename, 'w')
        json.dump(copy.copy(self.result), stream)

        # yaml export is optional
        if "yaml_output" in self.generation_config:
            if os.path.exists(self.generation_config["yaml_output"]):
            #    shutil.rmtree(self.config["yaml_output"]) #FIXME will fail if multiple test run concurrently
                pass
            else:
                os.makedirs(self.generation_config["yaml_output"])
            filename = os.path.join(self.generation_config["yaml_output"], self.name + ".yaml")
            stream = file(filename, 'w')
            yaml.dump(copy.copy(self.result), stream, default_flow_style=False)
