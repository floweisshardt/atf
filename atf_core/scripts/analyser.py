#!/usr/bin/env python
import rospy
import json
import yaml
import os
import unittest
import rostest
import copy
import shutil
import sys

from atf_core import ATFConfigurationParser
from atf_msgs.msg import TestblockState, TestblockTrigger


class Analyser:
    def __init__(self):
        self.ns = "/atf/"
        self.error = False

        # parse configuration
        atf_configuration_parser = ATFConfigurationParser()
        self.config = atf_configuration_parser.get_config()
        self.testblocks = atf_configuration_parser.create_testblocks(self.config, None, True)

        # monitor states for all testblocks
        self.testblock_states = {}
        for testblock in self.testblocks.keys():
            self.testblock_states[testblock] = TestblockState.INVALID

        # create trigger subscriber for all testblocks
        for testblock_name in self.testblocks.keys():
            topic = self.ns + testblock_name + "/trigger"
            #print "create subscriber to '%s' from testblock '%s'" % (topic, testblock_name)
            rospy.Subscriber(topic, TestblockTrigger, self.trigger_callback)

        rospy.loginfo("ATF analyser: started!")

    def trigger_callback(self, trigger):
        #print "trigger=", trigger

        if trigger.name not in self.config["test_config"].keys():
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

    def wait_for_all_testblocks_to_finish(self):
        # wait for all testblocks
        for testblock in self.testblock_states.keys():
            r = rospy.Rate(10)
            while not (self.testblock_states[testblock] == TestblockState.SUCCEEDED or self.testblock_states[testblock] == TestblockState.ERROR):
                #rospy.loginfo("Waiting for testblock '%s' to finish (current state is '%s')." % (testblock, self.testblock_states[testblock]))
                if rospy.is_shutdown():
                    break
                r.sleep()
                continue

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
        analyser.wait_for_all_testblocks_to_finish()
        #self.assertTrue(analyser.test_list, analyser.parsing_error_message)
        groundtruth_result, groundtruth_error_message, result = analyser.get_result()
        analyser.export_to_file(result)
        if groundtruth_result != None:
            self.assertTrue(groundtruth_result, groundtruth_error_message)


if __name__ == '__main__':
    rospy.init_node('test_analysing')
    if "standalone" in sys.argv:
        analyser = Analyser()
        analyser.wait_for_all_testblocks_to_finish()
        groundtruth_result, groundtruth_error_message, result = analyser.get_result()
        if groundtruth_result != None:
            rospy.logerr("groundtruth_result: '%s', groundtruth_error_message: '%s'", str(groundtruth_result), groundtruth_error_message)
        analyser.export_to_file(result)
    else:
        rostest.rosrun("atf_core", 'analysing', TestAnalysing, sysargs=None)
