#!/usr/bin/env python

from atf_msgs.msg import TestblockResult, TestblockStatus

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

        testblock_result = TestblockResult()
        testblock_result.name = self.name
        testblock_result.groundtruth_result = None

        if self.status == TestblockStatus.ERROR:
            testblock_result.groundtruth_result = False
            testblock_result.groundtruth_error_message = "An error occured during analysis of testblock '%s', no useful results available."%self.name
            print testblock_result.groundtruth_error_message
        else:
            print '>>', testblock_result.name
            #print "testblock.metrics=", self.metric_handles
            for metric_handle in self.metric_handles:
                # get result
                metric_result = metric_handle.get_result()

                # append result
                testblock_result.results.append(metric_result)

                # aggregate result
                if metric_result.groundtruth_result != None and not metric_result.groundtruth_result:
                    testblock_result.groundtruth_result = False
                    testblock_result.groundtruth_error_message += "\n     - metric '%s': %s"%(metric_result.name, metric_result.groundtruth_error_message)
                    #print testblock_result.groundtruth_error_message
                if testblock_result.groundtruth_result == None and metric_result.groundtruth_result:
                    testblock_result.groundtruth_result = True

        if len(testblock_result.results) == 0:
            raise ATFAnalyserError("Analysing failed, no testblock result available for testblock '%s'."%testblock_result.name)

        #print "\ntestblock_result:\n", testblock_result
        return testblock_result

class ATFTestblockError(Exception):
    pass
