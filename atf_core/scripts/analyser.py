#!/usr/bin/env python
import json
import yaml
import os
import progressbar
import rosbag
import rostest
import copy
import shutil
import sys
import time
import unittest

from atf_core import ATFConfigurationParser
from atf_msgs.msg import TestblockStatus, TestblockStatus


class Analyser:
    def __init__(self, package_name):
        print "ATF analyser: started!"
        self.ns = "/atf/"
        self.error = False

        # parse configuration
        self.configuration_parser = ATFConfigurationParser(package_name)
        self.tests = self.configuration_parser.get_tests()
        #self.testblocks = self.configuration_parser.create_testblocks(self.config, None, True)

        #print "self.config", self.config
        #print "self.testblocks", self.testblocks

        # monitor states for all testblocks
        #self.testblock_states = {}
        #for testblock in self.testblocks.keys():
        #    self.testblock_states[testblock] = TestblockStatus.INVALID

        start_time = time.time()
        #self.files = self.config["test_name"]#"/tmp/atf_test_app_time/data/ts0_c0_r0_e0_0.bag" # TODO get real file names from test config
        #print "self.files", self.files
        #files = self.get_file_paths(os.path.dirname(self.files), os.path.basename(self.files))
        i = 1
        for test in self.tests:
            inputfile = os.path.join(test.generation_config["bagfile_output"] + test.name + ".bag")
            print "Processing test %i/%i: %s"%(i,len(self.tests),test.name)
            try:
                bag = rosbag.Bag(inputfile)
            except rosbag.bag.ROSBagException as e:
                print "FATAL empty bag file", e
                continue
            if bag.get_message_count() == 0:
                print "FATAL empty bag file"
                continue
            bar = progressbar.ProgressBar(maxval=bag.get_message_count(), \
                    widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
            bar.start()
            j=0
            count_error=0

            try:
                for topic, raw_msg, t in bag.read_messages(raw=True):
                    try:
                        msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
                        msg = pytype()
                        msg.deserialize(serialized_bytes)
                        j+=1
                        for testblock in test.testblocks:
                            #print "testblock", testblock.name
                            #print "testblock.metric_handles", testblock.metric_handles
                            for metric_handle in testblock.metric_handles:
                                if topic == "/atf/status" and msg.name == testblock.name:
                                    #print "topic match for testblock '%s'"%testblock.name
                                    #print "testblock status for testblock '%s':"%testblock.name, testblock.status
                                    testblock.status = msg.status
                                    if testblock.status == TestblockStatus.ACTIVE:
                                        #print "calling start on metric", metric_handle
                                        metric_handle.start(msg.stamp)
                                    elif testblock.status == TestblockStatus.SUCCEEDED:
                                        #print "calling stop on metric", metric_handle
                                        metric_handle.stop(msg.stamp)
                                else:
                                    metric_handle.update(topic, msg, t)
                    #bar.update(j)
                    except StopIteration as e:
                        print "stop iterator", e
                        break
                    except Exception as e:
                        print "Exception", e
                        count_error += 1
                        continue
            except Exception as e:
            #except StopIteration as e:
                print "FATAL exception in bag file", type(e), e
                continue
            bar.finish()

            #for testblock in test.testblocks:
                #print "---testblock status for testblock '%s':"%testblock.name, testblock.status
            
            # check states for all testblocks
            for testblock in test.testblocks:
                if testblock.status == TestblockStatus.SUCCEEDED:
                    continue
                elif testblock.status == TestblockStatus.ERROR:
                    raise ATFAnalyserError("Testblock '%s' finished with ERROR." % testblock)
                else:
                    raise ATFAnalyserError("Testblock '%s' did not reach an end state before analyser finished (state is '%s'). Probably an error occured outside of monitored testblocks." % (testblock.name, testblock.status))
            
            # get result for each testblock
            overall_test_result = {}
            for testblock in test.testblocks:
                result = testblock.get_result()
                #print "result for testblock '%s' is"%testblock.name, result
                overall_test_result[testblock.name] = result

            test.result = overall_test_result
            #print "test.result:", test.result
            
            # export overall test result to file
            self.configuration_parser.export_to_file(test.result, os.path.join(test.generation_config["json_output"], test.name + ".json"))
            self.configuration_parser.export_to_file(test.result, os.path.join(test.generation_config["yaml_output"], test.name + ".yaml"))
            
            print "%d errors detected during test processing"%count_error
            i += 1
        
        #export test list
        test_list = self.configuration_parser.get_test_list()
        self.configuration_parser.export_to_file(test_list, os.path.join(test.generation_config["json_output"], "test_list.json"))
        self.configuration_parser.export_to_file(test_list, os.path.join(test.generation_config["yaml_output"], "test_list.yaml"))

        try:
            print "Processing tests took %s min"%str( round((time.time() - start_time)/60.0,4 ))
        except:
            pass

        print "ATF analyser: done!"

    def get_file_paths(self, dir, prefix):
        result = []
        for subdir, dirs, files in os.walk(dir):
            for file in files:
                full_path = os.path.join(subdir, file)
                if file.startswith(prefix):
                    result.append((file,full_path))
        result.sort()
        return result

    def get_result(self):
        result = {}
        overall_groundtruth_result = None
        overall_groundtruth_error_message = "groundtruth missmatch for: "

        for test in self.tests:
            #print "test =", test
            #print "test.name =", test.name
            for testblock in test.testblocks:
                #print "testblock =", testblock
                #print "testblock.name =", testblock.name
                #print "testbloc.status =", testblock.status
                if testblock.status == TestblockStatus.ERROR:
                    print "An error occured during analysis of testblock '%s', no useful results available."%testblock.name
                    result.update({testblock.name: {"status": "error"}})
                else:
                    #print "testblock.metrics=", testblock.metrics
                    for metric_handle in testblock.metric_handles:
                        #print "metric_handle=", metric_handle
                        metric_result = metric_handle.get_result()
                        #print "metric_result=", metric_result
                        if metric_result is not False:
                            (metric_name, data, groundtruth_result, groundtruth, groundtruth_epsilon, details) = metric_result
                            if testblock.name not in result:
                                result[testblock.name] = {}
                            if metric_name not in result[testblock.name]:
                                result[testblock.name][metric_name] = []
                            result[testblock.name][metric_name].append({"data":data, "groundtruth_result": groundtruth_result, "groundtruth": groundtruth, "groundtruth_epsilon": groundtruth_epsilon, "details": details})
                            if groundtruth_result == None:
                                pass
                            elif not groundtruth_result:
                                overall_groundtruth_result = False
                                overall_groundtruth_error_message += testblock.name + "(" + metric_name + ": data=" + str(data) + ", groundtruth=" + str(groundtruth) + "+-" + str(groundtruth_epsilon) + " details:" + str(details) + "); "
                            #print "overall_groundtruth_result =", overall_groundtruth_result
                            #print "overall_groundtruth_error_message =", overall_groundtruth_error_message
                        else:
                            raise ATFAnalyserError("No results for testblock '%s'" % (testblock.name))

        if result == {}:
            raise ATFAnalyserError("Analysing failed, no result available.")
        
        # overwrite overall_groundtruth_error_message if all tests are OK
        if overall_groundtruth_result == None or overall_groundtruth_result:
            overall_groundtruth_result = True
            overall_groundtruth_error_message = "All tests OK"

        return overall_groundtruth_result, overall_groundtruth_error_message, result
    
    def print_result(self, result):
        (overall_groundtruth_result, overall_groundtruth_error_message, result_details) = result
        #print "overall_groundtruth_result =", overall_groundtruth_result
        if overall_groundtruth_result:
            print "\n"
            print "********************"
            print "*** ALL TESTS OK ***"
            print "********************"
            print "\n"
        else:
            print "\n"
            print "*************************"
            print "*** SOME TESTS FAILED ***"
            print "*************************"
            print "\n"
        print "*** overall_groundtruth_error_message ***\n", overall_groundtruth_error_message
        print "\n"
        print "*** result details ***\n", result_details

class ATFAnalyserError(Exception):
    pass


class TestAnalysing(unittest.TestCase):
    def test_Analysing(self):
        analyser = Analyser(sys.argv[1])
        result = analyser.get_result()
        analyser.print_result(result)
        (overall_groundtruth_result, overall_groundtruth_error_message, result_details) = result
        self.assertTrue(overall_groundtruth_result, overall_groundtruth_error_message)

if __name__ == '__main__':
    print "analysing for package", sys.argv[1]
    if "standalone" in sys.argv:
        analyser = Analyser(sys.argv[1])
        result = analyser.get_result()
        analyser.print_result(result)

    else:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=sys.argv)
