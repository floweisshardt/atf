#!/usr/bin/env python
import rospy
import yaml

from atf_msgs.msg import *


class ATFServer:
    def __init__(self):
        rospy.Subscriber("/atf/test_status", TestStatus, self.status_update_callback, queue_size=1)
        self.total = 0

    def status_update_callback(self, data):
        print data.test_name, data.status_recording, data.status_analysing, data.total
        """
        test_list = self.load_data("/home/fmw-fk/test.yaml")
        self.total = data.total
        if data.test_name not in test_list:
            test_list[data.test_name]["list"].append(data.test_name)
            test_list[data.test_name]["status"].append([data.status_recording, data.status_analysing])
        self.save_data("/home/fmw-fk/test.yaml", test_list)
        """

    @staticmethod
    def load_data(filename):
        rospy.loginfo("Reading data from yaml file...")

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)

        return doc

    @staticmethod
    def save_data(filename, data):
        rospy.loginfo("Writing data to yaml file...")
        stream = file(filename, 'w')
        yaml.dump(data, stream)


if __name__ == "__main__":
    rospy.init_node('atf_server')
    ATFServer()
    rospy.spin()
