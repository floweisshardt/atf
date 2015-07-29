#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import json
import os
from copy import copy

from atf_msgs.msg import Status


class ATF:
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False
        test_name = rosparam.get_param("/suite_name")[0] + rosparam.get_param("/suite_name")[4] + rosparam.get_param(
            "/suite_name").split("_")[1] + "_" + rosparam.get_param("/test_name")[0] + rosparam.get_param(
            "/test_name").split("_")[1]

        if not os.path.exists(rospkg.RosPack().get_path("atf_presenter") + "/data/"):
            os.makedirs(rospkg.RosPack().get_path("atf_presenter") + "/data/")

        self.filename = rospkg.RosPack().get_path("atf_presenter") + "/data/" + test_name + ".json"

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
                self.export_to_file()
                break

    def export_to_file(self):
        doc = {}
        for item in self.testblocks:
            name = item.testblock
            for metric in item.metrics:
                (t, m, data) = metric.get_result()
                if name not in doc:
                    doc.update({name: {"timestamp": round(t, 3)}})
                    doc.update({name: {m: data}})
                else:
                    doc[name].update({"timestamp": round(t, 3)})
                    doc[name].update({m: data})

        stream = file(self.filename, 'w')
        json.dump(doc, stream)
