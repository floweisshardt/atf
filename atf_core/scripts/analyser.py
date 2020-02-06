#!/usr/bin/env python
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
from atf_msgs.msg import AtfResult, TestblockStatus


class Analyser:
    def __init__(self, package_name):
        print "ATF analyser: started!"
        start_time = time.time()
        self.ns = "/atf/"
        self.error = False

        # parse configuration
        self.configuration_parser = ATFConfigurationParser(package_name)
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
        atf_result.groundtruth_result = None
        atf_result.groundtruth_error_message = "Failed ATF tests:"
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
            if test_result.groundtruth_result != None and not test_result.groundtruth_result:
                atf_result.groundtruth_result = False
                atf_result.groundtruth_error_message += "\n - test '%s' (%s, %s, %s, %s): %s"%(test_result.name, test_result.robot, test_result.env, test_result.test_config, test_result.testblockset, test_result.groundtruth_error_message)
            if atf_result.groundtruth_result == None and test_result.groundtruth_result:
                atf_result.groundtruth_result = True

        if len(atf_result.results) == 0:
            raise ATFAnalyserError("Analysing failed, no atf result available.")

        # export overall atf result to file
        #print "\natf_result:\n", atf_result
        self.configuration_parser.export_to_file(atf_result, os.path.join(test.generation_config["txt_output"], "atf_result.txt"))
        return atf_result

    def print_result(self, atf_result):
        if atf_result.groundtruth_result != None and not atf_result.groundtruth_result:
            print "\n"
            print "*************************"
            print "*** SOME TESTS FAILED ***"
            print "*************************"
            print atf_result.groundtruth_error_message
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
            if result.groundtruth_result:
                print "test '%s' (%s, %s, %s, %s): succeeded"%(result.name, result.robot, result.env, result.test_config, result.testblockset)
            else:
                print "test '%s' (%s, %s, %s, %s): failed"%(result.name, result.robot, result.env, result.test_config, result.testblockset)

class ATFAnalyserError(Exception):
    pass


class TestAnalysing(unittest.TestCase):
    def test_analysing(self):
        analyser = Analyser(sys.argv[1])
        atf_result = analyser.get_result()
        analyser.print_result_details(atf_result)
        analyser.print_result(atf_result)
        if atf_result.groundtruth_result != None:
            self.assertTrue(atf_result.groundtruth_result, atf_result.groundtruth_error_message)

if __name__ == '__main__':
    print "analysing for package", sys.argv[1]
    if "standalone" in sys.argv:
        analyser = Analyser(sys.argv[1])
        atf_result = analyser.get_result()
        if "verbose" in sys.argv:
            analyser.print_result_details(atf_result)
        analyser.print_result(atf_result)
    else:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=sys.argv)
