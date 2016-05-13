#!/usr/bin/env python
import rospy
import yaml
import os
import unittest
import rostest
import rospkg

from atf_core import ATF, Testblock
import atf_metrics


class Analyser:
    def __init__(self):
        self.parsing_error_message = ""
        self.test_list = self.create_test_list()
        self.atf = ATF(self.test_list)

    def create_test_list(self):
        if not rospy.get_param("/analysing/result_yaml_output") == "":
            if not os.path.exists(rospy.get_param("/analysing/result_yaml_output")):
                os.makedirs(rospy.get_param("/analysing/result_yaml_output"))

        if not os.path.exists(rospy.get_param("/analysing/result_json_output")):
            os.makedirs(rospy.get_param("/analysing/result_json_output"))

        test_config_path = rospy.get_param("/analysing/test_config_file")
        config_data = self.load_data(test_config_path)[rospy.get_param("/analysing/test_config")]
        metrics_data = self.load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")

        testblock_list = []

        for testblock_name in config_data:
            metrics = []

            for metric in config_data[testblock_name]:
                metric_return = getattr(atf_metrics, metrics_data[metric]["handler"])() \
                    .parse_parameter(config_data[testblock_name][metric])
                if type(metric_return) == list:
                    for metric in metric_return:
                        metrics.append(metric)
                else:
                    self.parsing_error_message = "no valid metric configuration for metric '" + metric + "' in testblock '" + testblock_name + "'"
                    rospy.logerr(self.parsing_error_message)
                    return False

            testblock_list.append(Testblock(testblock_name, metrics))

        return testblock_list

    def load_data(self, filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
            return doc


class TestAnalysing(unittest.TestCase):
    def test_Analysing(self):
        analyser = Analyser()
        self.assertTrue(analyser.test_list, analyser.parsing_error_message)
        groundtruth_result, groundtruth_error_message = analyser.atf.check_states()
        if groundtruth_result != None:
            self.assertTrue(groundtruth_result, groundtruth_error_message)


if __name__ == '__main__':
    rospy.init_node('test_analysing')
    rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=None)
