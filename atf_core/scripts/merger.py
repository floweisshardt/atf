#!/usr/bin/env python
import rospy
import yaml
import unittest
import rostest
import os
import copy
import json

from atf_core import ATFConfigurationParser

class Merger():
    def __init__(self):
        self.ns = "/atf/"
        self.result = False

        self.atf_configuration_parser = ATFConfigurationParser()
        self.config = self.atf_configuration_parser.get_config()

    def merge(self):
        test_list = self.atf_configuration_parser.load_data(os.path.join(self.config["json_output"], "test_list.json"))
        #print "test_list=", test_list
        for test in test_list:
            #print "test=", test
            for test_name, test_data in test.items():
                #print "test_name=", test_name
                #print "test_data=", test_data
                subtests = test_data['subtests']
                #print "subtests=", subtests
                test_data_merged = {}
                for subtest in subtests:
                    #print "subtest=", subtest
                    subtest_data = self.atf_configuration_parser.load_data(os.path.join(self.config["json_output"], subtest + ".json"))
                    #print "subtest_data=", subtest_data
                    if subtest_data != None:
                        for testblock_name, testblock_data in subtest_data.items():
                            #print "testblock_name=", testblock_name
                            #print "testblock_data=", testblock_data
                            if testblock_name == "error":
                                rospy.logwarn("subtest '%s' has an error (error_message: '%s'), skipping...", subtest, testblock_data)
                                #TODO: mark subtest as error, so that presenter can show status information
                                continue
                            for metric_name, metric_data_list in testblock_data.items():
                                #print "metric_name=", metric_name
                                #print "metric_data_list=", metric_data_list
                                for metric_data in metric_data_list:
                                    #print "metric_data=", metric_data
                                    #print "metric_data['data']=", metric_data['data']

                                    # check if entry exists
                                    if testblock_name not in test_data_merged:
                                        # create new testblock entry
                                        #print "create new entry for testblock '" + testblock_name + "'"
                                        test_data_merged[testblock_name] = {}
                                    if metric_name not in test_data_merged[testblock_name]:
                                        # create new metric entry
                                        #print "create new entry for metric '" + metric_name + "' in testblock '" + testblock_name + "'"
                                        test_data_merged[testblock_name][metric_name] = []
                                        new_metric_data = copy.deepcopy(metric_data)
                                        new_metric_data['data'] = {}
                                        new_metric_data['data']['values'] = [metric_data['data']]
                                        test_data_merged[testblock_name][metric_name].append(new_metric_data)
                                    #print "test_data_merged0=", test_data_merged
                                    else:
                                        # entry already exists
                                        #print "entry for metric '" + metric_name + "' in testblock '" + testblock_name + "' already exists"

                                        # check if merging is possible, if not: append
                                        is_in, element_number = self.is_in_metric_data_list(copy.deepcopy(metric_data), copy.deepcopy(test_data_merged[testblock_name][metric_name]))
                                        if is_in:
                                            print "--> merge", metric_data['data'], "into element_number:", element_number
                                            # merge values
                                            test_data_merged[testblock_name][metric_name][element_number]['data']['values'].append(metric_data['data'])
                                            # merge groundtruth_result (take the worst result)
                                            test_data_merged[testblock_name][metric_name][element_number]['groundtruth_result'] = test_data_merged[testblock_name][metric_name][element_number]['groundtruth_result'] and metric_data['groundtruth_result']
                                        else:
                                            #print "--> append"
                                            new_metric_data = copy.deepcopy(metric_data)
                                            new_metric_data['data'] = {}
                                            new_metric_data['data']['values'] = [metric_data['data']]
                                            #print "new_metric_data=", new_metric_data
                                            #print "append to:", test_data_merged[testblock_name]
                                            test_data_merged[testblock_name][metric_name].append(new_metric_data)
                                    #print "test_data_merged=", test_data_merged

                #print "test_data_merged before average=", test_data_merged

                # calculate min/max/average
                for testblock_name, testblock_data in test_data_merged.items():
                    #print "testblock_data=", testblock_data
                    for metric_name, metric_data_list in testblock_data.items():
                        #print "metric_data_list=", metric_data_list
                        for i in range(len(metric_data_list)):
                            #print "i=", i
                            #print "test_data_merged[testblock_name][metric_name][i]['data']['values']=", test_data_merged[testblock_name][metric_name][i]['data']['values']
                            test_data_merged[testblock_name][metric_name][i]['data']['min'] = min(test_data_merged[testblock_name][metric_name][i]['data']['values'])
                            test_data_merged[testblock_name][metric_name][i]['data']['max'] = max(test_data_merged[testblock_name][metric_name][i]['data']['values'])
                            test_data_merged[testblock_name][metric_name][i]['data']['average'] = round(sum(test_data_merged[testblock_name][metric_name][i]['data']['values'])/len(test_data_merged[testblock_name][metric_name][i]['data']['values']), 3)

                #print "test_data_merged after average=", test_data_merged

                # write to file
                filename = os.path.join(self.config["json_output"], "merged_" + test_name + ".json")
                stream = file(filename, 'w')
                json.dump(copy.copy(test_data_merged), stream)

                filename = os.path.join(self.config["yaml_output"], "merged_" + test_name + ".yaml")
                if not filename == "":
                    stream = file(filename, 'w')
                    yaml.dump(copy.copy(test_data_merged), stream, default_flow_style=False)
        self.result = True

    def is_in_metric_data_list(self, data, data_list):
        counter = 0
        for dat in data_list:
            is_same = self.is_same_metric_data(copy.deepcopy(data), copy.deepcopy(dat))
            if is_same:
                return True, counter
            counter += 1
        return False, None

    def is_same_metric_data(self, data1, data2):
        data1.pop('data')
        data1.pop('groundtruth_result')
        data2.pop('data')
        data2.pop('groundtruth_result')
        if data1 == data2:
            return True
        else:
            return False


class TestMerging(unittest.TestCase):
    def test_merging_results(self):
        merger = Merger()
        merger.merge()
        self.assertTrue(merger.result, "Could not merge results.")

if __name__ == '__main__':
    rospy.init_node('test_merging')
    rostest.rosrun("atf_core", 'merging', TestMerging, sysargs=None)
