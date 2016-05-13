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
        self.testblock_1.start()
        self.testblock_3.start()
        rospy.sleep(3)
        self.testblock_1.stop()
        self.testblock_2.start()
        rospy.sleep(5)
        self.testblock_2.stop()
        self.testblock_3.stop()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('test_name')
    rostest.rosrun('application', 'recording', Test, sysargs=None)
