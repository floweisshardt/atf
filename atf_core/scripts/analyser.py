#!/usr/bin/env python
import argparse
import json
import os
import progressbar
import rosbag
import rospy
import rostest
import sys
import time
import traceback
import unittest
import yaml

from atf_core.configuration_parser import ATFConfigurationParser
from atf_metrics.error import ATFAnalyserError
from atf_msgs.msg import AtfResult, TestResult, TestblockResult, MetricResult, TestblockStatus, Groundtruth
from atf_metrics import metrics_helper

class Analyser:
    def __init__(self, package_name, test_generation_config_file="atf/test_generation_config.yaml"):
        print("ATF analyser: started!")
        self.ns = "/atf/"
        self.package_name = package_name

        # parse configuration
        self.configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)
        self.tests = self.configuration_parser.get_tests()

    def analyse(self, dry_run):
        if dry_run:
            return

        # generate results
        start_time = time.time()
        i = 1
        for test in self.tests:
            inputfile = os.path.join(test.generation_config["bagfile_output"] + test.name + ".bag")
            print("Processing test %i/%i: %s"%(i, len(self.tests), test.name))
            try:
                bag = rosbag.Bag(inputfile)
            except rosbag.bag.ROSBagException as e:
                print("ERROR empty bag file", e)
                i += 1
                continue
            except IOError as e:
                print("Error bag file not found", e)
                i += 1
                continue
            if bag.get_message_count() == 0:
                print("ERROR empty bag file")
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
                        print("stop iterator", type(e), e)
                        break
                    except Exception as e:
                        print("general Exception in ATF analyser", type(e), e)
                        print(traceback.format_exc())
                        count_error += 1
                        continue
                    bar.update(j)
            except Exception as e:
                print("FATAL exception in bag file", type(e), e)
                print(traceback.format_exc())
                continue
            bar.finish()



            print("%d errors detected during test processing"%count_error)
            i += 1

        #export test list
        test_list = self.configuration_parser.get_test_list()
        self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["txt_output"], "test_list.txt"))
        #self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["json_output"], "test_list.json"))
        #self.configuration_parser.export_to_file(test_list, os.path.join(self.configuration_parser.generation_config["yaml_output"], "test_list.yaml"))

        try:
            print("Processing tests took %s sec"%str(round((time.time() - start_time), 4)))
        except:
            pass

        print("ATF analyser: done!")

    def get_file_paths(self, directory, prefix):
        result = []
        for subdir, dirs, files in os.walk(directory):
            for filename in files:
                full_path = os.path.join(subdir, filename)
                if filename.startswith(prefix):
                    result.append((filename, full_path))
        result.sort()
        return result

    def get_result(self, dry_run):
        atf_result = AtfResult()
        atf_result.header.stamp = rospy.Time(time.time())
        atf_result.name = self.package_name
        atf_result.result = None
        atf_result.error_message = "All tests OK"

        for test in self.tests:
            # get result
            if dry_run:
                test_result = TestResult()
                test_result.name = test.name
                test_result.test_config = test.test_config_name
                test_result.robot = test.robot_name
                test_result.env = test.env_name
                test_result.testblockset = test.testblockset_name
                test_result.result = True
                test_result.error_message ="dry run"
            else:
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
                atf_result.error_message += "\n - test '%s' (%s, %s, %s, %s): %s"%(test_result.name, test_result.test_config, test_result.robot, test_result.env, test_result.testblockset, test_result.error_message)
            if atf_result.result == None and test_result.result:
                atf_result.result = True

        if len(atf_result.results) == 0:
            raise ATFAnalyserError("Analysing failed, no atf result available.")

        # export overall atf result to file
        #print "\natf_result:\n", atf_result
        self.configuration_parser.export_to_file(atf_result, os.path.join(test.generation_config["txt_output"], "atf_result.txt"))
        self.configuration_parser.export_to_file(atf_result, os.path.join(test.generation_config["txt_output"], "atf_result.bag"))

        # aggregate results
        atf_result_aggregated = self.aggregate_results(atf_result)
        #print "\natf_result_aggregated:\n", atf_result_aggregated
        self.configuration_parser.export_to_file(atf_result_aggregated, os.path.join(test.generation_config["txt_output"], "atf_result_aggregated.txt"))
        self.configuration_parser.export_to_file(atf_result_aggregated, os.path.join(test.generation_config["txt_output"], "atf_result_aggregated.bag"))

        return atf_result

    def aggregate_results(self, atf_result):
        test_list = self.configuration_parser.get_test_list()

        ret = self.configuration_parser.get_sorted_plot_dicts(atf_result, "", "", "")

        mbt = ret['mbt']
        mbt_aggregated = {}
        for metric in list(mbt.keys()):
            #print "m=", metric
            if metric not in list(mbt_aggregated.keys()):
                mbt_aggregated[metric] = {}
            for testblock in list(mbt[metric].keys()):
                #print "  b=", testblock
                if testblock not in list(mbt_aggregated[metric].keys()):
                    mbt_aggregated[metric][testblock] = {}
                for tl_tests in test_list:
                    #print "tl_tests=", tl_tests
                    for tl_test in list(tl_tests.keys()):
                        #print "    tl_test=", tl_test
                        metric_result = MetricResult()
                        status = TestblockStatus.SUCCEEDED
                        groundtruth_result = Groundtruth.SUCCEEDED
                        groundtruth_error_message = ""
                        details = []
                        for test in list(mbt[metric][testblock].keys()):
                            if test.startswith(tl_test):
                                # aggregate status SUCCEEDED from every metric_result
                                if mbt[metric][testblock][test].status != TestblockStatus.SUCCEEDED:
                                    status = TestblockStatus.ERROR

                                # aggregate data from every metric_result
                                data = mbt[metric][testblock][test].data
                                stamp = data.stamp
                                # check if data is set (not all default values anymore)
                                if data.stamp == rospy.Time(0) and data.data == 0:
                                    stamp = rospy.Time(0) # mark metric result as invalid by settimg timestamp to zero
                                metric_result.series.append(data)

                                # aggregate groundtruth from every metric_result
                                groundtruth = mbt[metric][testblock][test].groundtruth
                                if groundtruth.result != Groundtruth.SUCCEEDED:
                                    groundtruth_result = Groundtruth.FAILED
                                    if groundtruth_error_message != "":
                                        groundtruth_error_message += "\n"
                                    groundtruth_error_message += "groundtruth missmatch in subtest %s"%(test)
                                
                                # aggregate details from every metric_result
                                details = details + mbt[metric][testblock][test].details

                        if len(metric_result.series) == 0: # no matching substest found
                            continue

                        metric_result.groundtruth = groundtruth
                        metric_result.groundtruth.result = groundtruth_result
                        metric_result.groundtruth.error_message = groundtruth_error_message

                        metric_result.name          = mbt[metric][testblock][test].name
                        metric_result.unit          = mbt[metric][testblock][test].unit
                        metric_result.mode          = MetricResult.SPAN_MEAN # aggregated metrics are always SPAN_MEAN
                        metric_result.status        = status
                        # metric_result.series is set above
                        metric_result.data.stamp    = stamp
                        metric_result.data.data     = metrics_helper.get_mean(metric_result.series)
                        metric_result.min           = metrics_helper.get_min(metric_result.series)
                        metric_result.max           = metrics_helper.get_max(metric_result.series)
                        metric_result.mean          = metric_result.data.data
                        metric_result.std           = metrics_helper.get_std(metric_result.series)
                        # metric_result.groundtruth is set above
                        metric_result.details       = details
                        mbt_aggregated[metric][testblock][tl_test] = metric_result

        # convert mbt to tbm
        tbm = {}
        for metric in list(mbt_aggregated.keys()):
            #print "m=", metric
            for testblock in list(mbt_aggregated[metric].keys()):
                #print "  b=", testblock
                for test in list(mbt_aggregated[metric][testblock].keys()):
                    #print "    t=", test
                    if test not in list(tbm.keys()):
                        tbm[test] = {}
                    if testblock not in list(tbm[test].keys()):
                        tbm[test][testblock] = {}
                    tbm[test][testblock][metric] = mbt_aggregated[metric][testblock][test]

        # convert tbm to atf_result_aggregated
        atf_result_aggregated = AtfResult()
        atf_result_aggregated.header = atf_result.header
        atf_result_aggregated.name = atf_result.name
        atf_result_aggregated.result = True
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
                    if metric_result.groundtruth.result != Groundtruth.SUCCEEDED:
                        testblock_result.result = False
                        testblock_result.error_message += "\n     - metric '%s': %s"%(metric_result.name, metric_result.groundtruth.error_message)
                
                test_result.results.append(testblock_result)
                # aggregate testblock result
                if testblock_result.result == False:
                    test_result.result = False
                    test_result.error_message += "\n   - testblock '%s': %s"%(testblock_result.name, testblock_result.error_message)

            atf_result_aggregated.results.append(test_result)
            # aggregate test result
            if test_result.result == False:
                atf_result_aggregated.result = False
                atf_result_aggregated.error_message += "\n - test '%s' (%s, %s, %s, %s): %s"%(test_result.name, test_result.test_config, test_result.robot, test_result.env, test_result.testblockset, test_result.error_message)

        return atf_result_aggregated

    def print_result(self, atf_result):
        if atf_result.result != None and not atf_result.result:
            print("\n")
            print("*************************")
            print("*** SOME TESTS FAILED ***")
            print("*************************")
            print(atf_result.error_message)
            self.print_result_summary(atf_result)
        else:
            print("\n")
            print("********************")
            print("*** ALL TESTS OK ***")
            print("********************")
            self.print_result_summary(atf_result)

    def print_result_details(self, atf_result):
        print("\n")
        print("**********************")
        print("*** result details ***")
        print("**********************")
        print(atf_result)

    def print_result_summary(self, atf_result):
        print("\n")
        print("**********************")
        print("*** result summary ***")
        print("**********************")
        for result in atf_result.results:
            if result.result:
                print("test '%s' (%s, %s, %s, %s): succeeded"%(result.name, result.test_config, result.robot, result.env, result.testblockset))
            else:
                print("test '%s' (%s, %s, %s, %s): failed"%(result.name, result.test_config, result.robot, result.env, result.testblockset))

class TestAnalysing(unittest.TestCase):
    def test_analysing(self):
        analyser = Analyser(args.pkg, args.test_generation_config_file)
        analyser.analyse(args.dry_run)
        atf_result = analyser.get_result(args.dry_run)
        analyser.print_result(atf_result)
        if atf_result.result != None:
            self.assertTrue(atf_result.result, atf_result.error_message)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Manual exection of ATF analysing phase.', formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('pkg', type=str,
                        help='test package name')
    parser.add_argument('-g', dest='test_generation_config_file',
                        default='atf/test_generation_config.yaml',
                        help='path to test_generation_config file, relative to package root')
    parser.add_argument('-v', dest='verbose', action='count',
                        help='verbose output')
    parser.add_argument('-e', dest='execute_as_test', action='count',
                        help='execute as rostest')
    parser.add_argument('-d', dest='dry_run', action='count',
                        help='execute dry run')

    args, unknown = parser.parse_known_args()
    if args.execute_as_test:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing)
    else:
        args = parser.parse_args() # strictly parse only known arguments again. will raise an error if unknown arguments are specified
        print("analysing for package '%s' and test generation config file '%s'" %(args.pkg, args.test_generation_config_file))
        analyser = Analyser(args.pkg, args.test_generation_config_file)
        analyser.analyse(args.dry_run)
        atf_result = analyser.get_result(args.dry_run)
        if args.verbose:
            analyser.print_result_details(atf_result)
        analyser.print_result(atf_result)
