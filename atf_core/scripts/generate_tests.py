#!/usr/bin/env python
import yaml
import rospkg
import os
import lxml.etree
import lxml.builder
import shutil
import sys

from xml.etree import ElementTree
from xml.dom import minidom

import atf_core
from atf_core.configuration_parser import ATFConfigurationParser

class GenerateTests:
    def __init__(self, arguments):
        self.ns = "/atf/"

        expected_arg_length = 5
        if len(arguments) != expected_arg_length:
            error_message = "ATF configuration Error: invalid number of arguments for generating tests. Number of arguments is %d but should be %d"%(len(arguments), expected_arg_length)
            print(error_message)
            sys.exit(1)

        self.package_name = arguments[1]
        self.test_generation_config_file = arguments[2]
        self.package_src_path = arguments[3]
        self.package_bin_path = arguments[4]
        self.atf_configuration_parser = ATFConfigurationParser(self.package_src_path, self.test_generation_config_file, skip_metrics=True)
        self.tests = self.atf_configuration_parser.get_tests()

        self.test_list = {}

        test_generation_config_file_name = self.test_generation_config_file
        # replace directory "/" with "_"
        test_generation_config_file_name = test_generation_config_file_name.replace("/", "_")
        # replace "*.yaml" with "*_yaml"
        test_generation_config_file_name = test_generation_config_file_name.replace(".", "_")
        self.test_generated_path = os.path.join(self.package_bin_path, "test_generated", test_generation_config_file_name)
        self.create_folders()

    def create_folders(self):
        # delete old test_generated directory and create new one
        if os.path.exists(self.test_generated_path):
            shutil.rmtree(self.test_generated_path)
        shutil.copyfile(self.package_src_path + "/package.xml", self.package_bin_path + "/package.xml")
        os.makedirs(self.test_generated_path)

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
            xml_test({'test-name': "cleaning",
                    'pkg': "atf_core",
                    'type': "cleaner.py",
                    'time-limit': "60",
                    'args': self.package_name + " -g " + self.test_generation_config_file + " -e"})
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
                xml_arg(name="execute_as_test", default="true"),
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
            self.append_parameters(test_record, xml_rosparam, test.robot_config, "additional_parameters")           
            # env params
            self.append_parameters(test_record, xml_rosparam, test.env_config, "additional_parameters")
            # test params
            self.append_parameters(test_record, xml_rosparam, test.test_config, "additional_parameters")

            if ("app_launch_file" in test.generation_config):
                incl = xml_include(file="$(find " + self.package_name + ")/" + test.generation_config["app_launch_file"])
                # robot args
                self.append_arguments(incl, xml_arg, test.robot_config, "additional_arguments")
                # env args
                self.append_arguments(incl, xml_arg, test.env_config, "additional_arguments")
                # test args
                self.append_arguments(incl, xml_arg, test.test_config, "additional_arguments")
                
                test_record.append(incl)

            test_record.append(
                xml_node(
                    pkg=self.package_name,
                    type=test.generation_config['app_executable'],
                    name="$(anon atf_application)",
                    required="true",
                    output="screen")),
            test_record.append(
                xml_test({'if':'$(arg execute_as_test)',
                        'pkg':'atf_core',
                        'type':'sm_test.py',
                        'test-name': "atf_recording_" + test.name,
                        'time-limit': str(test.generation_config["time_limit_recording"]),
                        'required': "true",
                        'args':'execute_as_test'}))
            test_record.append(
                xml_node({'unless':'$(arg execute_as_test)',
                        'pkg':'atf_core',
                        'type':'sm_test.py',
                        'name': "atf_recording_" + test.name,
                        'required': "true",
                        'output':'screen'}))

            xmlstr = minidom.parseString(ElementTree.tostring(test_record)).toprettyxml(indent="    ")
            filepath = os.path.join(self.test_generated_path, "recording_" + test.name) + ".test"
            with open(filepath, "w") as f:
                f.write(xmlstr)

        # Analysing
        test_analyse = xml_launch(
            xml_test({'test-name': "analysing",
                    'pkg': "atf_core",
                    'type': "analyser.py",
                    'time-limit': str(self.atf_configuration_parser.generation_config["time_limit_analysing"]),
                    'required': "true",
                    'args': self.package_name + " -g " + self.test_generation_config_file + " -e"})
        )

        xmlstr = minidom.parseString(ElementTree.tostring(test_analyse)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "analysing.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)

        # Uploading
        test_upload = xml_launch()
        if test.generation_config["upload_data"]:
            test_upload.append(
                xml_test({'test-name': "uploading_data",
                        'pkg': "atf_core",
                        'type': "test_dropbox_uploader.py",
                        'time-limit': str(self.atf_configuration_parser.generation_config["time_limit_uploading"]),
                        'args': "-f " + os.path.join(self.package_src_path, "atf/.dropbox_uploader_config") + " upload " + self.atf_configuration_parser.generation_config["bagfile_output"] + " " + os.path.join(self.package_name, "data")}))

        if test.generation_config["upload_result"]:
            test_upload.append(
                xml_test({'test-name': "uploading_results",
                        'pkg': "atf_core",
                        'type': "test_dropbox_uploader.py",
                        'time-limit': str(self.atf_configuration_parser.generation_config["time_limit_uploading"]),
                        'args': "-f " + os.path.join(self.package_src_path, "atf/.dropbox_uploader_config") + " upload " + self.atf_configuration_parser.generation_config["txt_output"] + " " + os.path.join(self.package_name, "results")}))

        xmlstr = minidom.parseString(ElementTree.tostring(test_upload)).toprettyxml(indent="    ")
        filepath = os.path.join(self.test_generated_path, "uploading.test")
        with open(filepath, "w") as f:
            f.write(xmlstr)

    def append_parameters(self, xml_parent, xml_type, dictionary, key):
        if (dictionary != None) and (key in dictionary):
            elements = dictionary[key]
            if len(elements) > 0:
                for element in elements:
                    for name, value in list(element.items()):
                        xml_parent.append(xml_type(str(value), param=str(name), subst_value="True"))

    def append_arguments(self, xml_parent, xml_type, dictionary, key):
        if (dictionary != None) and (key in dictionary):
            elements = dictionary[key]
            if len(elements) > 0:
                for element in elements:
                    for name, value in list(element.items()):
                        xml_parent.append(xml_type(name=str(name), value=str(value)))

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
    print("ATF: Test generation started...")
    GenerateTests(sys.argv).generate_tests()
    print("ATF: ...Test generation done!")
