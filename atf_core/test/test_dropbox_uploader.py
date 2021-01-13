#!/usr/bin/env python
import unittest
import rospy
import rostest
import os
import sys

class Test(unittest.TestCase):
    def setUp(self):
        print(sys.argv)
        if len(sys.argv) < 6:
            rospy.logerr("wrong number of arguments: expected 5, got %d", len(sys.argv))
            sys.exit(1)
        self.du_config_file = sys.argv[1] + " " + sys.argv[2]
        self.du_command = sys.argv[3]
        self.src_path = sys.argv[4]
        self.package_name = sys.argv[5]

    def tearDown(self):
        pass

    def test_dropbox_uploader(self):
        command = "rosrun atf_core dropbox_uploader.sh" + " " + self.du_config_file + " " + self.du_command + " " + self.src_path + " " + self.package_name
        rospy.loginfo("calling %s", command)
        os.system(command)

if __name__ == '__main__':
    rospy.init_node('test_name')
    rostest.rosrun('application', 'test_dropbox', Test)
