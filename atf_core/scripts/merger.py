#!/usr/bin/env python
import rospy
import yaml
import unittest
import rostest
import os
import copy
import json
import yaml
import numpy

class Merger():
    def __init__(self):
        self.result = False
        self.merging_error_message = ""
        self.json_output = rospy.get_param("merging/result_json_output")
        self.yaml_output = rospy.get_param("merging/result_yaml_output")

    def merge(self):
        test_list = self.load_data(os.path.join(self.json_output, "test_list.json"))
        #print "test_list=", test_list
        for test in test_list:
            print "test=", test
            for test_name, test_data in test.items():
                #print "test_name=", test_name
                #print "test_data=", test_data
                subtests = test_data['subtests']
                #print "subtests=", subtests
                test_data_merged = {}
                for subtest in subtests:
                    subtest_data = subtests
                    #print "subtest=", subtest
                    subtest_data = self.load_data(os.path.join(self.json_output, subtest + ".json"))
                    #print "subtest_data=", subtest_data
                    for testblock_name, testblock_data in subtest_data.items():
                        #print "testblock_name=", testblock_name
                        #print "testblock_data=", testblock_data
                        for metric_name, metric_data_list in testblock_data.items():
                            #print "metric_name=", metric_name
                            #print "metric_data_list=", metric_data_list
                            for metric_data in metric_data_list:
                                #print "metric_data=", metric_data
                                #print "metric_data['data']=", metric_data['data']
                                
                                # check if entry exists
                                if testblock_name not in test_data_merged:
                                    test_data_merged[testblock_name] = {}
                                    if metric_name not in test_data_merged:
                                        # create new entry
                                        test_data_merged[testblock_name][metric_name] = {}
                                        test_data_merged[testblock_name][metric_name] = copy.deepcopy(metric_data)
                                        test_data_merged[testblock_name][metric_name]['data'] = {}
                                        test_data_merged[testblock_name][metric_name]['data']['values'] = []
                                print "test_data_merged0=", test_data_merged
                                test_data_merged[testblock_name][metric_name]['data']['values'].append(metric_data['data'])
                                print "test_data_merged1=", test_data_merged

                print "test_data_merged final=", test_data_merged
                
                # calculate min/max/average
                for testblock_name, testblock_data in test_data_merged.items():
                    print "testblock_data=", testblock_data
                    for metric_name, metric_data in testblock_data.items():
                        print "metric_data=", metric_data
                        test_data_merged[testblock_name][metric_name]['data']['min'] = min(metric_data['data']['values'])
                        test_data_merged[testblock_name][metric_name]['data']['max'] = max(metric_data['data']['values'])
                        test_data_merged[testblock_name][metric_name]['data']['average'] = round(sum(metric_data['data']['values'])/len(metric_data['data']['values']),3)
                print "test_data_merged after average=", test_data_merged
                
                # write to file
                filename = self.json_output + test_name + ".json"
                stream = file(filename, 'w')
                json.dump(copy.copy(test_data_merged), stream)

                filename = self.yaml_output + test_name + ".yaml"
                if not filename == "":
                    stream = file(filename, 'w')
                    yaml.dump(copy.copy(test_data_merged), stream, default_flow_style=False)
        self.result = True

    def load_data(self, filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
            return doc

class TestMerging(unittest.TestCase):
    def test_MergingResults(self):
        merger = Merger()
        merger.merge()
        self.assertTrue(merger.result, merger.merging_error_message)

if __name__ == '__main__':
    rospy.init_node('test_merging')
    rostest.rosrun("atf_core", 'merging', TestMerging, sysargs=None)
