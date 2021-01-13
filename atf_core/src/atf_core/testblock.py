#!/usr/bin/env python

from atf_metrics.error import ATFAnalyserError
from atf_msgs.msg import TestblockResult, TestblockStatus, Groundtruth

class Testblock:
    def __init__(self, name, metric_handles, recorder_handle):

        self.name = name
        self.metric_handles = metric_handles
        self.recorder_handle = recorder_handle
        self.trigger = None
        self.timestamp = None
        self.exception = None
        self.status = TestblockStatus.INACTIVE

    def get_result(self):

        testblock_result = TestblockResult()
        testblock_result.name = self.name
        testblock_result.result = None

        if self.status == TestblockStatus.ERROR:
            testblock_result.result = False
            testblock_result.error_message = "An error occured during analysis of testblock '%s', no results available."%self.name
            print(testblock_result.error_message)
        else:
            #print "testblock.metrics=", self.metric_handles
            for metric_handle in self.metric_handles:
                # get result
                metric_result = metric_handle.get_result()

                # append result
                testblock_result.results.append(metric_result)

                # aggregate result
                if metric_result.groundtruth.result != Groundtruth.SUCCEEDED:
                    testblock_result.result = False
                    testblock_result.error_message += "\n     - metric '%s': %s"%(metric_result.name, metric_result.groundtruth.error_message)
                    #print testblock_result.groundtruth_error_message
                if testblock_result.result == None and metric_result.groundtruth.result == Groundtruth.SUCCEEDED:
                    testblock_result.result = True

        if testblock_result.result == None:
            raise ATFAnalyserError("Analysing failed, testblock result is None for testblock '%s'."%testblock_result.name)

        #print "\ntestblock_result:\n", testblock_result
        return testblock_result
