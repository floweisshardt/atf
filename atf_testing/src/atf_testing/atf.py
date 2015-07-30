#!/usr/bin/env python
import rospy
import rospkg
import rosparam
import json
import os
from threading import Thread
from atf_msgs.msg import Status


def threaded(fn):
    def wrapper(*args, **kwargs):
        Thread(target=fn, args=args, kwargs=kwargs).start()
    return wrapper


class ATF:
    @threaded
    def __init__(self, testblocks):

        self.testblocks = testblocks
        self.error = False
        self.testblock_error = {}
        self.test_name = rosparam.get_param("/test_name")

        if not os.path.exists(rospkg.RosPack().get_path("atf_presenter") + "/data/"):
            os.makedirs(rospkg.RosPack().get_path("atf_presenter") + "/data/")

    def wait_for_end(self):
        while not rospy.is_shutdown() and not self.error:
            for testblock in self.testblocks:
                try:
                    if testblock.get_state() == Status.ERROR:
                        self.testblock_error[testblock.testblock] = Status.ERROR
                        rospy.loginfo("An error occured during analysis in '" + testblock.testblock + "', no useful " +
                                      "results available.")
                        self.error = True
                        break
                except ValueError:
                    pass

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
        json.dump(doc, stream)
