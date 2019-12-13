#!/usr/bin/env python
import yaml
import json
import re
import itertools as it
import rospkg
import os
import lxml.etree
import lxml.builder
import shutil
import sys

from xml.etree import ElementTree
from xml.dom import minidom

from copy import deepcopy, copy

import atf_core

class GenerateTests:
    def __init__(self, arguments):
        self.ns = "/atf/"

        expected_arg_length = 5
        if len(arguments) != expected_arg_length:
            error_message = "ATF configuration Error: invalid number of arguments for generating tests. Number of arguments is %d but should be %d"%(len(arguments), expected_arg_length)
            print error_message
            sys.exit(1)

        self.package_name = arguments[1]
        self.test_generation_config_file = arguments[2]
        self.package_src_path = arguments[3]
        self.package_bin_path = arguments[4]
        self.atf_configuration_parser = atf_core.ATFConfigurationParser(self.package_src_path, self.test_generation_config_file, skip_metrics=True)
        self.tests = self.atf_configuration_parser.get_tests()

        self.test_list = {}

        self.test_generated_path = os.path.join(self.package_bin_path, "test_generated")
        self.test_generated_recording_path = os.path.join(self.test_generated_path, "recording")
        #self.test_generated_analysing_path = os.path.join(self.test_generated_path, "analysing")
        self.create_folders()

    def create_folders(self):
        # delete of test_generated directory and create new one
        if os.path.exists(self.test_generated_recording_path):
            shutil.rmtree(self.test_generated_path)
        shutil.copyfile(self.package_src_path + "/package.xml", self.package_bin_path + "/package.xml")
        os.makedirs(self.test_generated_path)
        os.makedirs(self.test_generated_recording_path)
        #os.makedirs(self.test_generated_analysing_path)

    def generate_tests(self):
        em = lxml.builder.ElementMaker()
        xml_launch = em.launch
        xml_arg = em.arg
        xml_include = em.include
        xml_test = em.test
        xml_node = em.node
        xml_param = em.param
        xml_rosparam = em.rosparam

        # Cleaning
        test_clean = xml_launch(
            #param(name=self.ns + "bag_output", value=self.generation_config["bagfile_output"]),
            #param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
            #param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
            xml_test({'test-name': "cleaning", 'pkg': "atf_core", 'type': "cleaner.py",
                'time-limit': "60", 'args': self.package_name})
        )
        xmlstr = minidom.parseString(ElementTree.tostring(test_clean)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "cleaning.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)


        # Recording
        for test in self.tests:
            #print "robot_config=", robot_config
            #print "robot_env_config=", robot_env_config
            #print "self.test_list[test_name]=", self.test_list[test_name]

            test_record = xml_launch(
                #arg(name="robot", value=self.test_list[test_name]["robot"]),
                #include(arg(name="test_status_list", value="$(find " + self.package_name + ")/test_status.yaml"),
                #        file="$(find atf_status_server)/launch/atf_status_server.launch"),
                xml_param(name=self.ns + "package_name", value=self.package_name),
                xml_param(name=self.ns + "test_generation_config_file", value=self.test_generation_config_file),
                xml_param(name=self.ns + "test_name", value=test.name),
                #param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
                #param(name=self.ns + "robot_config_name", value=self.test_list[test_name]["robot"]),
                #param(name=self.ns + "robot_env_config_name", value=self.test_list[test_name]["robot_env"]),
                #rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml")),
                #rosparam(param=self.ns + "robot_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["robot_config_path"], self.test_list[test_name]["robot"] + ".yaml")),
                #rosparam(param=self.ns + "robot_env_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["robot_env_config_path"], self.test_list[test_name]["robot_env"] + ".yaml")),
                #param(name=self.ns + "bagfile_output", value=self.generation_config["bagfile_output"]),
                #param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
                #param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
            )

            # robot params
            if "additional_parameters" in test.robot_config:
                if len(test.robot_config["additional_parameters"]) > 0:
                    for robot_param_name, robot_param_value in test.robot_config["additional_parameters"].items():
                        test_record.append(xml_rosparam(str(robot_param_value), param=str(robot_param_name), subst_value="True"))
            # robot_env params
            if "additional_parameters" in test.env_config:
                if len(test.env_config["additional_parameters"]) > 0:
                    for robot_env_param_name, robot_env_param_value in test.env_config["additional_parameters"].items():
                        test_record.append(xml_rosparam(str(robot_env_param_value), param=str(robot_env_param_name), subst_value="True"))

            if "app_launch_file" in test.generation_config:
                incl = xml_include(file="$(find " + self.package_name + ")/" + test.generation_config["app_launch_file"])
                # robot args
                if "additional_arguments" in test.robot_config:
                    if len(test.robot_config["additional_arguments"]) > 0:
                        for robot_arg_name, robot_arg_value in test.robot_config["additional_arguments"].items():
                            incl.append(xml_arg(name=str(robot_arg_name), value=str(robot_arg_value)))
                # robot_env args
                if "additional_arguments" in test.env_config:
                    if len(test.env_config["additional_arguments"]) > 0:
                        for robot_env_arg_name, robot_env_arg_value in test.env_config["additional_arguments"].items():
                            incl.append(xml_arg(name=str(robot_env_arg_name), value=str(robot_env_arg_value)))
                
                test_record.append(incl)

            test_record.append(xml_node(pkg=self.package_name, type=test.generation_config['app_executable'], name="$(anon application)", required="true", output="screen")),
            test_record.append(xml_test({'pkg':'atf_core', 'type':'sm_test.py', 'test-name': "recording_" + test.name,
                        'time-limit': str(test.generation_config["time_limit_recording"]), 'required': "true"}))

            xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_recording_path, "recording_" + test.name) + ".test"
            with open(filepath, "w") as f:
                f.write(xmlstr)

        # Analysing
        test_analyse = xml_launch(
            xml_test({'test-name': "analysing", 'pkg': "atf_core", 'type': "analyser.py",
                    'time-limit': str(self.atf_configuration_parser.generation_config["time_limit_analysing"]), 'required': "true", 'args': self.package_name})
        )

        xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "analysing.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)

        # Merging
        test_merge = xml_launch(
            #param(name=self.ns + "test_name", value=test_name),
            #param(name=self.ns + "test_config_name", value=self.test_list[test_name]["test_config"]),
            #rosparam(param=self.ns + "test_config", command="load", file="$(find " + self.package_name + ")/" + os.path.join(self.generation_config["test_config_path"], self.test_list[test_name]["test_config"] + ".yaml")),
            #param(name=self.ns + "yaml_output", value=self.generation_config["yaml_output"]),
            #param(name=self.ns + "json_output", value=self.generation_config["json_output"]),
            xml_test({'test-name': "merging", 'pkg': "atf_core", 'type': "merger.py",
                    'time-limit': "60", 'args': self.package_name})
        )
        xmlstr = minidom.parseString(ElementTree.tostring(test_merge)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "merging.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)

        # Uploading
        test_upload = xml_launch()
        if test.generation_config["upload_data"]:
            test_upload.append(
                xml_test({'test-name': "uploading_data", 'pkg': "atf_core", 'type': "test_dropbox_uploader.py",
                        'time-limit': str(self.time_limit_uploading), 'args': "-f " + os.path.join(self.package_src_path, "atf/.dropbox_uploader_config") + " upload " + self.bagfile_output + " " + os.path.join(self.package_name, "data")}))

        if test.generation_config["upload_result"]:
            test_upload.append(
                xml_test({'test-name': "uploading_results", 'pkg': "atf_core", 'type': "test_dropbox_uploader.py",
                        'time-limit': str(self.atf_configuration_parser.generation_config["time_limit_uploading"]), 'args': "-f " + os.path.join(self.package_src_path, "atf/.dropbox_uploader_config") + " upload " + self.atf_configuration_parser.generation_config["txt_output"] + " " + os.path.join(self.package_name, "results")}))

        xmlstr = minidom.parseString(ElementTree.tostring(test_upload)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "uploading.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)





    @staticmethod
    def remove_pkgname(text, pkgname):
        return text[len(pkgname):]

    def get_path(self, path):
        try:
            rospkg.RosPack().get_path(path.split("/")[0]) + self.remove_pkgname(path, path.split("/")[0])
        except rospkg.common.ResourceNotFound:
            return path
        else:
            return rospkg.RosPack().get_path(path.split("/")[0]) + self.remove_pkgname(path, path.split("/")[0])


if __name__ == '__main__':
    print "ATF: Test generation started..."
    GenerateTests(sys.argv).generate_tests()
    print "ATF: ...Test generation done!"
