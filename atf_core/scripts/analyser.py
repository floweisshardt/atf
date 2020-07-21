#!/usr/bin/env python
import copy
import json
import os
import progressbar
import rosbag
import rospy
import rostest
import sys
import time
import unittest
import yaml

from atf_core import ATFConfigurationParser
from atf_msgs.msg import AtfResult, TestResult, TestblockResult, MetricResult, TestblockStatus
from atf_metrics import metrics_helper

class Analyser:
    def __init__(self, package_name, test_generation_config_file = "atf/test_generation_config.yaml"):
        print "ATF analyser: started!"
        start_time = time.time()
        self.ns = "/atf/"
        self.error = False

        # parse configuration
        self.configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)
        self.tests = self.configuration_parser.get_tests()

        # generate results
        i = 1
        for test in self.tests:
            inputfile = os.path.join(test.generation_config["bagfile_output"] + test.name + ".bag")
            print "Processing test %i/%i: %s"%(i, len(self.tests), test.name)
            try:
                bag = rosbag.Bag(inputfile)
            except rosbag.bag.ROSBagException as e:
                print "ERROR empty bag file", e
                i += 1
                continue
            except IOError as e:
                print "Error bag file not found", e
                i += 1
                continue
            if bag.get_message_count() == 0:
                print "ERROR empty bag file"
                i += 1
                continue
            bar = progressbar.ProgressBar(maxval=bag.get_message_count(), \
                    widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
            bar.start()
            j = 0
            count_error = 0

            try:
                for topic, raw_msg, t in bag.read_messages(raw=True):
                    try:
                        msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
                        msg = pytype()
                        msg.deserialize(serialized_bytes)
                        j += 1
                        for testblock in test.testblocks:
                            #print "testblock", testblock.name
                            #print "testblock.metric_handles", testblock.metric_handles
                            for metric_handle in testblock.metric_handles:
                                if topic == "/atf/status" and msg.name == testblock.name:
                                    testblock.status = msg.status
                                    if testblock.status == TestblockStatus.ACTIVE:
                                        #print "calling start on metric", metric_handle
                                        metric_handle.start(msg)
                                    elif testblock.status == TestblockStatus.SUCCEEDED:
                                        #print "calling stop on metric", metric_handle
                                        metric_handle.stop(msg)
                                else:
                                    metric_handle.update(topic, msg, t)
                    except StopIteration as e:
                        print "stop iterator", type(e), e
                        break
                    except Exception as e:
                        print "general Exception in ATF analyser", type(e), e
                        count_error += 1
                        continue
                    bar.update(j)
            except Exception as e:
                print "FATAL exception in bag file", type(e), e
                continue
            bar.finish()



            print "%d errors detected during test processing"%count_error
            i += 1

        #export test list
        test_list = self.configuration_parser.get_test_list()
        self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["txt_output"], "test_list.txt"))
        #self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["json_output"], "test_list.json"))
        #self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["yaml_output"], "test_list.yaml"))

        try:
            print "Processing tests took %s sec"%str(round((time.time() - start_time), 4))
        except:
            pass

        print "ATF analyser: done!"

    def get_file_paths(self, directory, prefix):
        result = []
        for subdir, dirs, files in os.walk(directory):
            for filename in files:
                full_path = os.path.join(subdir, filename)
                if filename.startswith(prefix):
                    result.append((filename, full_path))
        result.sort()
        return result

    def get_result(self):
        atf_result = AtfResult()
        atf_result.header.stamp = rospy.Time(time.time())
        atf_result.result = None
        atf_result.error_message = "All tests OK"
        for test in self.tests:
            # get result
            test_result = test.get_result()

            # export test result to file
            self.configuration_parser.export_to_file(test_result, os.path.join(test.generation_config["txt_output"], test.name + ".txt"))
            #self.configuration_parser.export_to_file(test_result, os.path.join(test.generation_config["json_output"], test.name + ".json")) # ROS message object is not JSON serialisable
            #self.configuration_parser.export_to_file(test_result, os.path.join(test.generation_config["yaml_output"], test.name + ".yaml")) # ROS message object is not correctly serialized to yaml

            # append testresult to overall atf result
            atf_result.results.append(test_result)

            # aggregate result
            if test_result.result != None and not test_result.result:
                # check if there are already failed tests in atf_result
                if atf_result.result == None:
                    atf_result.error_message = "Failed ATF tests:"
                atf_result.result = False
                atf_result.error_message += "\n - test '%s' (%s, %s, %s, %s): %s"%(test_result.name, test_result.robot, test_result.env, test_result.test_config, test_result.testblockset, test_result.error_message)
            if atf_result.result == None and test_result.result:
                atf_result.result = True

        if len(atf_result.results) == 0:
            raise ATFAnalyserError("Analysing failed, no atf result available.")

        # export overall atf result to file
        #print "\natf_result:\n", atf_result
        self.configuration_parser.export_to_file(atf_result, os.path.join(test.generation_config["txt_output"], "atf_result.txt"))
        self.configuration_parser.export_to_file(atf_result, os.path.join(test.generation_config["txt_output"], "atf_result.bag"))

        # merge results
        atf_result_merged = self.merge_results(atf_result)
        #print "\natf_result_merged:\n", atf_result_merged
        self.configuration_parser.export_to_file(atf_result_merged, os.path.join(test.generation_config["txt_output"], "atf_result_merged.txt"))
        self.configuration_parser.export_to_file(atf_result_merged, os.path.join(test.generation_config["txt_output"], "atf_result_merged.bag"))

        return atf_result

    def merge_results(self, atf_result):
        test_list = self.configuration_parser.get_test_list()

        ret = self.configuration_parser.get_sorted_plot_dicts(atf_result, "", "", "")

        mbt = ret['mbt']
        mbt_merged = {}
        for metric in mbt.keys():
            #print "m=", metric
            if metric not in mbt_merged.keys():
                mbt_merged[metric] = {}
            for testblock in mbt[metric].keys():
                #print "  b=", testblock
                if testblock not in mbt_merged[metric].keys():
                    mbt_merged[metric][testblock] = {}
                for tl_tests in test_list:
                    #print "tl_tests=", tl_tests
                    for tl_test in tl_tests.keys():
                        #print "    tl_test=", tl_test
                        metric_result = MetricResult()
                        groundtruth_result = True
                        groundtruth_error_message = ""
                        for test in mbt[metric][testblock].keys():
                            if test.startswith(tl_test):
                                metric_result.series.append(mbt[metric][testblock][test].data)

                                # aggregate groundtruth for every metric
                                groundtruth = mbt[metric][testblock][test].groundtruth
                                if groundtruth.result == False:
                                    groundtruth_result = False
                                    if groundtruth_error_message != "":
                                        groundtruth_error_message += "\n"
                                    groundtruth_error_message += "groundtruth missmatch in subtest %s"%(test)

                        metric_result.groundtruth = groundtruth
                        metric_result.groundtruth.result = groundtruth_result
                        metric_result.groundtruth.error_message = groundtruth_error_message

                        metric_result.name          = mbt[metric][testblock][test].name
                        metric_result.mode          = MetricResult.SPAN # merged metrics are always SPAN
                        metric_result.started       = mbt[metric][testblock][test].started
                        metric_result.finished      = mbt[metric][testblock][test].finished
                        # metric_result.series is set above
                        metric_result.data.stamp    = atf_result.header.stamp
                        metric_result.data.data     = metrics_helper.get_mean(metric_result.series)
                        metric_result.min           = metrics_helper.get_min(metric_result.series)
                        metric_result.max           = metrics_helper.get_max(metric_result.series)
                        metric_result.mean          = metric_result.data.data
                        metric_result.std           = metrics_helper.get_std(metric_result.series)
                        # metric_result.groundtruth is set above
                        metric_result.details       = mbt[metric][testblock][test].details
                        mbt_merged[metric][testblock][tl_test] = metric_result

        # convert mbt to tbm
        tbm = {}
        for metric in mbt_merged.keys():
            #print "m=", metric
            for testblock in mbt_merged[metric].keys():
                #print "  b=", testblock
                for test in mbt_merged[metric][testblock].keys():
                    #print "    t=", test
                    if test not in tbm.keys():
                        tbm[test] = {}
                    if testblock not in tbm[test].keys():
                        tbm[test][testblock] = {}
                    tbm[test][testblock][metric] = mbt_merged[metric][testblock][test]

        # convert tbm to atf_result_merged
        atf_result_merged = AtfResult()
        atf_result_merged.header = atf_result.header
        atf_result_merged.result = True
        for test in sorted(tbm.keys()):
            test_result = TestResult()
            test_result.name = test
            test_result.result = True

            # find test metadata in atf_result
            for t in atf_result.results:
                if t.name.startswith(test):
                    test_result.test_config = t.test_config
                    test_result.robot = t.robot
                    test_result.env = t.env
                    test_result.testblockset = t.testblockset
                    break

            for testblock in sorted(tbm[test].keys()):
                testblock_result = TestblockResult()
                testblock_result.name = testblock
                testblock_result.result = True
                for metric in sorted(tbm[test][testblock].keys()):
                    metric_result = tbm[test][testblock][metric]
                    testblock_result.results.append(metric_result)
                    # aggregate metric result
                    if metric_result.groundtruth.result == False:
                        testblock_result.result = False
                        testblock_result.error_message += "\n     - metric '%s': %s"%(metric_result.name, metric_result.groundtruth.error_message)
                
                test_result.results.append(testblock_result)
                # aggregate testblock result
                if testblock_result.result == False:
                    test_result.result = False
                    test_result.error_message += "\n   - testblock '%s': %s"%(testblock_result.name, testblock_result.error_message)

            atf_result_merged.results.append(test_result)
            # aggregate test result
            if test_result.result == False:
                atf_result_merged.result = False
                atf_result_merged.error_message += "\n - test '%s' (%s, %s, %s, %s): %s"%(test_result.name, test_result.robot, test_result.env, test_result.test_config, test_result.testblockset, test_result.error_message)

        return atf_result_merged

    def print_result(self, atf_result):
        if atf_result.result != None and not atf_result.result:
            print "\n"
            print "*************************"
            print "*** SOME TESTS FAILED ***"
            print "*************************"
            print atf_result.error_message
            self.print_result_summary(atf_result)
        else:
            print "\n"
            print "********************"
            print "*** ALL TESTS OK ***"
            print "********************"
            self.print_result_summary(atf_result)

    def print_result_details(self, atf_result):
        print "\n"
        print "**********************"
        print "*** result details ***"
        print "**********************"
        print atf_result

    def print_result_summary(self, atf_result):
        print "\n"
        print "**********************"
        print "*** result summary ***"
        print "**********************"
        for result in atf_result.results:
            if result.result:
                print "test '%s' (%s, %s, %s, %s): succeeded"%(result.name, result.robot, result.env, result.test_config, result.testblockset)
            else:
                print "test '%s' (%s, %s, %s, %s): failed"%(result.name, result.robot, result.env, result.test_config, result.testblockset)

class TestAnalysing(unittest.TestCase):
    def test_analysing(self):
        analyser = Analyser(package_name, test_generation_config_file)
        atf_result = analyser.get_result()
        analyser.print_result(atf_result)
        if atf_result.result != None:
            self.assertTrue(atf_result.result, atf_result.error_message)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        package_name = sys.argv[1]
        test_generation_config_file = "atf/test_generation_config.yaml"
    elif len(sys.argv) > 2:
        package_name = sys.argv[1]
        test_generation_config_file = sys.argv[2]
    else:
        print "ERROR: please specify a test package"
        print "usage: rosrun atf_core analyser.py <<ATF TEST PACKAGE>> [<<TEST_GENERATION_CONFIG_FILE>>]"
        sys.exit(1)
    print "analysing for package '%s' and test generation config file '%s'" %(package_name, test_generation_config_file)

    if "execute_as_test" in sys.argv:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing)
    else:
        analyser = Analyser(package_name, test_generation_config_file)
        atf_result = analyser.get_result()
        if "verbose" in sys.argv:
            analyser.print_result_details(atf_result)
        analyser.print_result(atf_result)
