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
    def __init__(self):
        self.ns = "/atf/"
        self.error = False

        # parse configuration
        atf_configuration_parser = ATFConfigurationParser()
        self.tests = atf_configuration_parser.get_tests()
        #self.testblocks = atf_configuration_parser.create_testblocks(self.config, None, True)

        #print "self.config", self.config
        #print "self.testblocks", self.testblocks

        # monitor states for all testblocks
        #self.testblock_states = {}
        #for testblock in self.testblocks.keys():
        #    self.testblock_states[testblock] = TestblockState.INVALID

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
                            if topic == "atf/status" and msg.name == testblock.name:
                                #print "topic match for testblock '%s'"%testblock.name
                                testblock.status = msg.status
                                print "testblock status for testblock '%s':"%testblock.name, testblock.status
                                if testblock.status == TestblockStatus.ACTIVE:
                                    print "testblock is active"
                                    print "testblock.metric_handles", testblock.metric_handles
                                    for metric_handle in testblock.metric_handles:
                                        print "calling start on metric", metric_handle
                                        metric_handle.start(msg.stamp)
                                elif testblock.status == TestblockStatus.SUCCEEDED:
                                    print "testblock is succeeded"
                                    print "testblock.metric_handles", testblock.metric_handles
                                    for metric_handle in testblock.metric_handles:
                                        print "calling stop on metric", metric_handle
                                        metric_handle.stop(msg.stamp)
                        #bar.update(j)
                    except StopIteration as e:
                        print "stop iterator", e
                        break
                    except Exception as e:
                        print "Exception", e
                    #except StopIteration as e:
                        count_error += 1
                        continue
            except Exception as e:
            #except StopIteration as e:
                print "FATAL exception in bag file", type(e), e
                continue
            bar.finish()

            for testblock in test.testblocks:
                print "---testblock status for testblock '%s':"%testblock.name, testblock.status
            
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
            test.export_to_file()
            
            print "%d errors detected during test processing"%count_error
            i += 1
            
        try:
            print "Processing tests took %s min"%str( round((time.time() - start_time)/60.0,4 ))
        except:
            pass


        print "ATF analyser: started!"

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

        for testblock_name, testblock in self.testblocks.items():
            if self.testblock_states[testblock_name] == TestblockState.ERROR:
                print "An error occured during analysis of testblock '%s', no useful results available."%testblock_name
                result.update({testblock.testblock_name: {"status": "error"}})
            else:
                #print "testblock.metrics=", testblock.metrics
                for metric_handle in testblock.metrics:
                    #print "metric_handle=", metric_handle
                    metric_result = metric_handle.get_result()
                    #print "metric_result=", metric_result
                    if metric_result is not False:
                        (metric_name, data, groundtruth_result, groundtruth, groundtruth_epsilon, details) = metric_result
                        if testblock_name not in result:
                            result[testblock_name] = {}
                        if metric_name not in result[testblock_name]:
                            result[testblock_name][metric_name] = []
                        result[testblock_name][metric_name].append({"data":data, "groundtruth_result": groundtruth_result, "groundtruth": groundtruth, "groundtruth_epsilon": groundtruth_epsilon, "details": details})
                        if groundtruth_result == None:
                            pass
                        elif not groundtruth_result:
                            overall_groundtruth_result = False
                            overall_groundtruth_error_message += testblock_name + "(" + metric_name + ": data=" + str(data) + ", groundtruth=" + str(groundtruth) + "+-" + str(groundtruth_epsilon) + " details:" + str(details) + "); "
                    else:
                        raise ATFAnalyserError("No result for metric '%s' in testblock '%s'" % (metric_name, testblock_name))

        if result == {}:
            raise ATFAnalyserError("Analysing failed, no result available.")
        return overall_groundtruth_result, overall_groundtruth_error_message, result
    

class ATFAnalyserError(Exception):
    pass


class TestAnalysing(unittest.TestCase):
    def test_Analysing(self):
        analyser = Analyser()
        for test in self.tests:
            print "test.name:", test.name
            if test.groundtruth_result != None:
                self.assertTrue(groundtruth_result, groundtruth_error_message)


if __name__ == '__main__':
    if "standalone" in sys.argv:
        analyser = Analyser()
    else:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=None)
