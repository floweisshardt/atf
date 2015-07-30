#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import json
import yaml

from atf_msgs.msg import Status
from copy import copy


class ATF:
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False
        self.testblock_error = {}
        self.test_name = rosparam.get_param("/test_name")

    def check_states(self):
        running_testblocks = copy(self.testblocks)
        while not rospy.is_shutdown() and not self.error:
            for testblock in self.testblocks:
                try:
                    if testblock.get_state() == Status.ERROR:
                        self.testblock_error[testblock.testblock] = Status.ERROR
                        rospy.loginfo("An error occured during analysis in '" + testblock.testblock + "', no useful " +
                                      "results available.")
                        self.error = True
                        break
                    elif testblock.get_state() == Status.FINISHED:
                        running_testblocks.remove(testblock)
                except ValueError:
                    pass
            if len(running_testblocks) == 0:
                break

        self.export_to_file()

    def export_to_file(self):
        doc = {}
        for item in self.testblocks:
            name = item.testblock
            if name in self.testblock_error:
                doc.update({name: {"status": "error"}})
            else:
                for metric in item.metrics:
                    if metric.get_result() is not False:
                        (t, m, data) = metric.get_result()
                        if name not in doc:
                            doc.update({name: {"timestamp": round(t, 3)}})
                            doc.update({name: {m: data}})
                        else:
                            doc[name].update({"timestamp": round(t, 3)})
                            doc[name].update({m: data})
                    else:
                        item.exit()
                        break

        filename = rospkg.RosPack().get_path("atf_presenter") + "/data/" + self.test_name + ".json"
        stream = file(filename, 'w')
        json.dump(copy(doc), stream)

        filename = rospkg.RosPack().get_path("atf_testing") + "/results/" + self.test_name + ".yaml"
        stream = file(filename, 'w')
        yaml.dump(doc, stream)
