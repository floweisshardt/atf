#!/usr/bin/env python
import rospy
import yaml
import rosparam

from atf_msgs.msg import *
from atf_status_server.srv import *


class ATFServer:
    def __init__(self):
        self.test_status_list = rosparam.get_param("status_list")

        sub = rospy.Subscriber("/atf/test_status", TestStatus, self.status_update_callback, queue_size=10)

        # Wait for publisher
        num_subscriber = sub.get_num_connections()
        while num_subscriber == 0:
            num_subscriber = sub.get_num_connections()

        rospy.Service("atf/get_test_status", GetTestStatus, self.status_service_callback)

        rospy.loginfo("ATF server started!")

    def status_update_callback(self, data):

        try:
            test_list = self.load_data(self.test_status_list)
        except IOError:
            test_list = {}

        # New test
        if data.test_name not in test_list:
            test_list[data.test_name] = {"status": [data.status_recording, data.status_analysing],
                                         "testblock": {}
                                         }
            test_list["total"] = data.total

        # Recording
        elif data.status_analysing == 0:
            test_list[data.test_name]["status"][0] = data.status_recording

        # Analysing
        elif data.status_analysing != 0:
            test_list[data.test_name]["status"][1] = data.status_analysing
            if len(data.testblock) != 0:
                test_list[data.test_name]["testblock"][data.testblock[0].name] = data.testblock[0].status

        self.save_data(self.test_status_list, test_list)

    @staticmethod
    def load_data(filename):
        with open(filename, 'r') as stream:
            doc = yaml.load(stream)

        return doc

    @staticmethod
    def save_data(filename, data):
        stream = file(filename, 'w')
        yaml.dump(data, stream, default_flow_style=False)

    def status_service_callback(self, req):
        if req:
            test_list = self.load_data(self.test_status_list)
            req_list = GetTestStatusResponse()

            for test in test_list:
                if test != "total":
                    data = TestStatus()
                    data.test_name = test
                    data.status_recording = test_list[test]["status"][0]
                    data.status_analysing = test_list[test]["status"][1]
                    for testblock in test_list[test]["testblock"]:
                        block = TestblockStatus()
                        block.name = testblock
                        block.status = test_list[test]["testblock"][testblock]
                        data.testblock.append(block)
                    data.total = test_list["total"]
                    req_list.status.append(data)

            return req_list


if __name__ == "__main__":
    rospy.init_node('atf_status_server')
    ATFServer()
    rospy.spin()
