#!/usr/bin/env python
import rospy
from copy import copy

from atf_msgs.msg import Status


class ATF:
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False

    def wait_for_end(self):
        _testblocks = copy(self.testblocks)
        while not rospy.is_shutdown() and not self.error:
            testblocks_temp = copy(_testblocks)
            for item in testblocks_temp:

                if item.get_state() == Status.ERROR:
                    rospy.loginfo("An error occured during analysis, no useful results available. State was " +
                                  str(item.get_state()))
                    self.error = True
                    break
                elif item.get_state() == Status.FINISHED:
                    _testblocks.remove(item)

            if len(_testblocks) == 0:
                self.print_results()
                break

    def print_results(self):
        rospy.loginfo("\n---- RESULTS ----")
        for item in self.testblocks:
            name = item.testblock
            rospy.loginfo("-- " + name + " --")
            for metric in item.metrics:
                rospy.loginfo(metric.get_result())
