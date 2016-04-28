#!/usr/bin/env python
import rospy
import yaml
import rosparam
import os
import unittest
import rostest
import rospkg

from atf_core import ATF, Testblock
import atf_metrics


class TestBuilder:
    def __init__(self):

        ATF(self.create_test_list()).check_states()

    def create_test_list(self):
        if not rosparam.get_param("/analysing/result_yaml_output") == "":
            if not os.path.exists(rosparam.get_param("/analysing/result_yaml_output")):
                os.makedirs(rosparam.get_param("/analysing/result_yaml_output"))

        if not os.path.exists(rosparam.get_param("/analysing/result_json_output")):
            os.makedirs(rosparam.get_param("/analysing/result_json_output"))

        test_config_path = rosparam.get_param("/analysing/test_config_file")
        config_data = self.load_data(test_config_path)[rosparam.get_param("/analysing/test_config")]
        metrics_data = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")

        testblock_list = []

        for testblock_name in config_data:
            metrics = []

            for metric in config_data[testblock_name]:
                metric_return = getattr(atf_metrics, metrics_data[metric]["handler"])() \
                    .parse_parameter(config_data[testblock_name][metric])
                if type(metric_return) == list:
                    for value in metric_return:
                        metrics.append(value)
                else:
                    metrics.append(metric_return)

            testblock_list.append(Testblock(testblock_name, metrics))

        return testblock_list

    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
            return doc


class TestAnalysing(unittest.TestCase):
    def test_Analysing(self):
        TestBuilder()


if __name__ == '__main__':
    rospy.init_node('test_analysing')
    rostest.rosrun("atf_core", 'test_analysing', TestAnalysing, sysargs=None)  # sysargs=['--text']
