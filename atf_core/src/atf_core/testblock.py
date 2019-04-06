#!/usr/bin/env python
import os
import rospy
import shutil

from atf_core import StateMachine#, ATFRecorder
from atf_msgs.msg import TestblockStatus, TestblockTrigger

class Testblock:
    def __init__(self, name, metric_handles, recorder_handle):

        self.name = name
        self.metric_handles = metric_handles
        self.recorder_handle = recorder_handle
        self.trigger = None
        self.timestamp = None
        self.exception = None
        self.atf_started = False
        self.status = None

    def get_result(self):
        result = {}
        #overall_groundtruth_result = None
        #overall_groundtruth_error_message = "groundtruth missmatch for: "

        if self.status == TestblockStatus.ERROR:
            print "An error occured during analysis of testblock '%s', no useful results available."%self.name
            result.update({name: {"status": "error"}})
        else:
            print "testblock.metrics=", self.metric_handles
            for metric_handle in self.metric_handles:
                print "metric_handle=", metric_handle
                metric_result = metric_handle.get_result()
                print "metric_result=", metric_result
                asdasdasdfasdff
                if metric_result is not False:
                    (metric_name, data, groundtruth_result, groundtruth, groundtruth_epsilon, details) = metric_result
                    if metric_name not in result:
                        result[metric_name] = []
                    result[metric_name].append({"data":data, "groundtruth_result": groundtruth_result, "groundtruth": groundtruth, "groundtruth_epsilon": groundtruth_epsilon, "details": details})
                    if groundtruth_result == None:
                        pass
                    elif not groundtruth_result:
                        overall_groundtruth_result = False
                        overall_groundtruth_error_message += self.name + "(" + metric_name + ": data=" + str(data) + ", groundtruth=" + str(groundtruth) + "+-" + str(groundtruth_epsilon) + " details:" + str(details) + "); "
                else:
                    raise ATFTestblockError("No result for metric '%s' in testblock '%s'" % (metric_name, self.name))

        #if result == {}:
        #    raise ATFAnalyserError("Analysing failed, no result available.")
        #return overall_groundtruth_result, overall_groundtruth_error_message, result
        return result

class ATFTestblockError(Exception):
    pass
