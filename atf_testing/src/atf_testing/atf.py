#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import yaml
from copy import copy

from atf_msgs.msg import Status


class ATF:
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False
        test = rosparam.get_param("/test_name")
        self.filename = rospkg.RosPack().get_path("atf_presenter") + "/data/" + test + ".yaml"

    def wait_for_end(self):
        _testblocks = copy(self.testblocks)
        while not rospy.is_shutdown() and not self.error:
            testblocks_temp = copy(_testblocks)
            for item in testblocks_temp:

                try:
                    if item.get_state() == Status.ERROR:
                        rospy.loginfo("An error occured during analysis, no useful results available. State was " +
                                      str(item.get_state()))
                        self.error = True
                        break
                    elif item.get_state() == Status.FINISHED:
                        _testblocks.remove(item)
                except ValueError:
                    pass

            if len(_testblocks) == 0:
                # self.print_results()
                self.export_to_yaml()
                break

    def print_results(self):
        rospy.loginfo("\n")
        rospy.loginfo("---- RESULTS ----")
        for item in self.testblocks:
            name = item.testblock
            rospy.loginfo("-- " + name + " --")
            for metric in item.metrics:
                rospy.loginfo(metric.get_result())

    def export_to_yaml(self):
        doc = {}
        for item in self.testblocks:
            name = item.testblock
            for metric in item.metrics:
                (t, m, data) = metric.get_result()
                if name not in doc:
                    doc.update({name: {"timestamp": t}})
                    for i in xrange(0, len(m)):
                        doc.update({name: {m[i]: data[i]}})
                else:
                    doc[name].update({"timestamp": t})
                    for i in xrange(0, len(m)):
                        doc[name].update({m[i]: data[i]})
        stream = file(self.filename, 'w')
        yaml.dump(doc, stream)
