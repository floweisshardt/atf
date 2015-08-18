#!/usr/bin/env python
import rospy
import rosparam
import json
import yaml
import rosgraph

from atf_msgs.msg import Status
from copy import copy


class ATF:
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False
        self.error_outside_testblock = False
        self.testblock_error = {}
        self.test_name = rosparam.get_param("/analysing/test_name")

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

        if rospy.is_shutdown():
            self.error_outside_testblock = True

        self.export_to_file()

    def export_to_file(self):
        doc = {}
        if self.error_outside_testblock:
            doc["Error"] = "An error occured outside monitored testblocks. Aborted analysis..."
        else:
            for item in self.testblocks:
                name = item.testblock_name
                if name in self.testblock_error:
                    doc.update({name: {"status": "error"}})
                else:
                    for metric in item.metrics:
                        result = metric.get_result()
                        if result is not False:
                            (t, m, data) = result
                            if name not in doc:
                                doc.update({name: {"timestamp": round(t, 3)}})
                                doc.update({name: {m: data}})
                            else:
                                doc[name].update({"timestamp": round(t, 3)})
                                doc[name].update({m: data})
                        else:
                            item.exit()
                            break

        filename = rosparam.get_param("/analysing/result_json_output") + self.test_name + ".json"
        stream = file(filename, 'w')
        json.dump(copy(doc), stream)

        filename = rosparam.get_param("/analysing/result_yaml_output") + self.test_name + ".yaml"
        if not filename == "":
            stream = file(filename, 'w')
            yaml.dump(doc, stream)
