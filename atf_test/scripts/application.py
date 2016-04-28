#!/usr/bin/python
import unittest
import yaml
from subprocess import call

import rospy
import rostest
import rostopic
from atf_recorder import RecordingManager


class Application:
    def __init__(self):
        self.testblock_1 = RecordingManager('testblock_1')
        self.testblock_2 = RecordingManager('testblock_2')
        self.testblock_3 = RecordingManager('testblock_3')

    def execute(self):
        # Example for recorder usage
        self.testblock_1.start()
        self.testblock_3.start()
        rospy.sleep(3)
        #self.recorder.error()
        self.testblock_1.stop()
        self.testblock_2.start()
        rospy.sleep(5)
        self.testblock_2.stop()
        self.testblock_3.stop()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

        robot_config = self.load_data(rospy.get_param('/robot_config'))
        self.topics = robot_config['wait_for_topics']
        self.services = robot_config['wait_for_services']

    def tearDown(self):
        call("killall gzclient", shell=True)
        call("killall gzserver", shell=True)

    def test_Recording(self):
        # Wait for topics and services
        for topic in self.topics:
            rospy.wait_for_message(topic, rostopic.get_topic_class(topic, blocking=True)[0], timeout=None)

        for service in self.services:
            rospy.wait_for_service(service, timeout=None)

        self.app.execute()

    @staticmethod
    def load_data(filename):
        rospy.loginfo("Reading data from yaml file...")

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)

        return doc


if __name__ == '__main__':
    rospy.init_node('test_name')
    rostest.rosrun('application', 'test_application', Test, sysargs=None)
