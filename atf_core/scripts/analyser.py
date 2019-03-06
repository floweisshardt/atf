#!/usr/bin/env python
import rospy
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
from atf_msgs.msg import TestblockState, TestblockTrigger


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
                        for testblock_name in test.test_config.keys():
                            if topic == self.ns + testblock_name + "/trigger":
                                self.trigger_callback(msg, test)
                        bar.update(j)
                    except StopIteration as e:
                        print "stop iterator", e
                        break
                    #except Exception as e:
                    except StopIteration as e:
                        count_error += 1
                        continue
            #except Exception as e:
            except StopIteration as e:
                print "FATAL exception in bag file", type(e), e
                continue
            bar.finish()
            print "%d errors detected during bag processing"%count_error
            i += 1
            
        try:
            print "Processing tests took %s min"%str( round((time.time() - start_time)/60.0,4 ))
        except:
            pass


        rospy.loginfo("ATF analyser: started!")

    def get_file_paths(self, dir, prefix):
        result = []
        for subdir, dirs, files in os.walk(dir):
            for file in files:
                full_path = os.path.join(subdir, file)
                if file.startswith(prefix):
                    result.append((file,full_path))
        result.sort()
        return result

    def trigger_callback(self, trigger, test):
        print "trigger=", trigger
        print "testblocks:", test.test_config.keys()

        if trigger.name not in test.test_config.keys():
            raise ATFAnalyserError("Testblock '%s' not in test_config." % trigger.name)

        if trigger.trigger == TestblockTrigger.PURGE:
            rospy.loginfo("Purging testblock '%s'", trigger.name)
            for metric in self.testblocks[trigger.name].metrics:
                metric.purge(trigger.stamp)
            self.testblock_states[trigger.name] = TestblockState.PURGED
        elif trigger.trigger == TestblockTrigger.START:
            rospy.loginfo("Starting testblock '%s'", trigger.name)
            for metric in self.testblocks[trigger.name].metrics:
                metric.start(trigger.stamp)
            self.testblock_states[trigger.name] = TestblockState.ACTIVE
        elif trigger.trigger == TestblockTrigger.PAUSE:
            rospy.loginfo("Pausing testblock '%s'", trigger.name)
            for metric in self.testblocks[trigger.name].metrics:
                metric.pause(trigger.stamp)
            self.testblock_states[trigger.name] = TestblockState.PAUSED
        elif trigger.trigger == TestblockTrigger.STOP:
            rospy.loginfo("Stopping testblock '%s'", trigger.name)
            for metric in self.testblocks[trigger.name].metrics:
                metric.stop(trigger.stamp)
            self.testblock_states[trigger.name] = TestblockState.SUCCEEDED
        else:
            raise ATFAnalyserError("Unknown trigger '%s' for testblock '%s'" % (str(trigger.trigger), trigger.name))

#    def wait_for_all_testblocks_to_finish(self):
#        # wait for all testblocks
#        for testblock in self.testblock_states.keys():
#            r = rospy.Rate(10)
#            while not (self.testblock_states[testblock] == TestblockState.SUCCEEDED or self.testblock_states[testblock] == TestblockState.ERROR):
#                #rospy.loginfo("Waiting for testblock '%s' to finish (current state is '%s')." % (testblock, self.testblock_states[testblock]))
#                if rospy.is_shutdown():
#                    break
#                r.sleep()
#                continue

        # check state of all testblocks
        for testblock, state in self.testblock_states.items():
            if state == TestblockState.SUCCEEDED:
                continue
            elif state == TestblockState.ERROR:
                raise ATFAnalyserError("Testblock '%s' finished with ERROR." % testblock)
            else:
                raise ATFAnalyserError("Testblock '%s' did not reach an end state before analyser finished (state is '%s'). Probably an error occured outside of monitored testblocks." % (testblock, state))

    def get_result(self):
        result = {}
        overall_groundtruth_result = None
        overall_groundtruth_error_message = "groundtruth missmatch for: "

        for testblock_name, testblock in self.testblocks.items():
            #print "testblock_name=", testblock_name

            #test_status = TestStatus()
            #test_status.test_name = self.test_name
            #test_status.status_analysing = 1

            #testblock_status = TestblockStatus()
            #testblock_status.name = testblock.testblock_name
            #testblock_status.status = testblock.get_state()

            #test_status.testblock.append(testblock_status)
            #test_status.total = self.number_of_tests

            #self.test_status_publisher.publish(test_status)

            if self.testblock_states[testblock_name] == TestblockState.ERROR:
                rospy.logerr("An error occured during analysis of testblock '%s', no useful results available.")
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

        #test_status = TestStatus()
        #test_status.test_name = self.test_name
        #test_status.status_analysing = 3
        #test_status.total = self.number_of_tests
        #self.test_status_publisher.publish(test_status)

        if result == {}:
            raise ATFAnalyserError("Analysing failed, no result available.")
        return overall_groundtruth_result, overall_groundtruth_error_message, result

    def export_to_file(self, result):
        # we'll always have json export
        if os.path.exists(self.config["json_output"]):
        #    shutil.rmtree(self.config["json_output"]) #FIXME will fail if multiple test run concurrently
            pass
        else:
            os.makedirs(self.config["json_output"])
        shutil.copyfile(os.path.join(self.config["test_generated_path"], "test_list.json"), os.path.join(self.config["json_output"], "test_list.json"))
        filename = os.path.join(self.config["json_output"], self.config["test_name"] + ".json")
        stream = file(filename, 'w')
        json.dump(copy.copy(result), stream)

        # yaml export is optional
        if "yaml_output" in self.config:
            if os.path.exists(self.config["yaml_output"]):
            #    shutil.rmtree(self.config["yaml_output"]) #FIXME will fail if multiple test run concurrently
                pass
            else:
                os.makedirs(self.config["yaml_output"])
            filename = os.path.join(self.config["yaml_output"], self.config["test_name"] + ".yaml")
            stream = file(filename, 'w')
            yaml.dump(copy.copy(result), stream, default_flow_style=False)

class ATFAnalyserError(Exception):
    pass


class TestAnalysing(unittest.TestCase):
    def test_Analysing(self):
        analyser = Analyser()
#        analyser.wait_for_all_testblocks_to_finish()
        #self.assertTrue(analyser.test_list, analyser.parsing_error_message)
        #groundtruth_result, groundtruth_error_message, result = analyser.get_result()
        analyser.export_to_file(result)
        if groundtruth_result != None:
            self.assertTrue(groundtruth_result, groundtruth_error_message)


if __name__ == '__main__':
    #rospy.init_node('test_analysing')
    if "standalone" in sys.argv:
        analyser = Analyser()
        #analyser.wait_for_all_testblocks_to_finish()
        #groundtruth_result, groundtruth_error_message, result = analyser.get_result()
        if groundtruth_result != None:
            rospy.logerr("groundtruth_result: '%s', groundtruth_error_message: '%s'", str(groundtruth_result), groundtruth_error_message)
        analyser.export_to_file(result)
    else:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=None)
