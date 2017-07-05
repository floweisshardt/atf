#!/usr/bin/python
import unittest
import rospy
import rostest
import tf
import math
import sys
import os
import rosnode
import time
import shutil

from atf_core import ATF
from simple_script_server import *

class Application:
    def __init__(self):
        # ATF code
        self.atf = ATF()

        # native app code
        self.pub_freq = 20.0 # Hz
        self.br = tf.TransformBroadcaster()
        rospy.sleep(1) #wait for tf broadcaster to get active (rospy bug?)
        self.sss = simple_script_server()
        self.filepath =  rospy.get_param("/atf/bagfile_output").replace("data/", "")

    def execute(self):
        rospy.sleep(1)
        self.initpose()
        rospy.sleep(5)
        self.atf.start("testblock_small")
#        poses = rospy.get_param("/script_server/base")
#        for pose in poses:
#            print "moving to pose", pose
#            sss.move("base",pose)
        #rosnode._rosnode_cmd_info(['rosnode', 'info', '/ipa_loc_feature_source_laser_node'])
        rospy.sleep(50)#750s
        self.atf.stop("testblock_small")
        # os.system('rosrun map_server map_saver map:=map_hmm -f '+self.filepath+'map_atf_stm'+str(time.time()))
        # os.system('rosrun map_server map_saver map:=ref_map -f '+self.filepath+'map_atf_ltm'+str(time.time()))
        #os.system('rosrun map_server map_saver map:=map_hmm -f '+self.filepath+'map_atf_stm'+str(time.time()))
        #os.system('rosrun map_server map_saver map:=overlay_cells_map -f '+self.filepath+'map_atf_overlay'+str(time.time()))
        #os.system('rosservice call /long_term_slam/backup')
        #rospy.sleep(5)
        #self.backup_changer()
        self.atf.shutdown()


    def backup_changer(self):
        count = 0
        countfile = self.filepath+"atf_count.txt"
        test_name = rospy.get_param("/atf/test_name")
        print test_name
        if os.path.isfile(countfile):
            with open(countfile, 'r+') as file:
                #print "file:",file
                #count = int(file.read())
                for line in file:
                    count = line
                print "count", count
                count = int(count)
                if (count > 1):
                    shutil.copy("/home/fmw-hb/.ipa_navigation/long_term_slam_backup/client/init_ipa.backup", "/home/fmw-hb/.ipa_navigation/long_term_slam_backup/client/long_term_slam.backup")
                    count = 0
                    print "backups copied!!!"
                else:
                    count += 1
                file.write(test_name+"\n"+str(count)+"\n")

        else:
            with open(countfile, 'w') as file:
                file.write(test_name+"\n"+str(count)+"\n")

    def initpose(self):
        pub_initialpose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        initialpose = PoseWithCovarianceStamped()
        initialpose.header.stamp = rospy.Time.now()
        print "stamp: "+str( initialpose.header.stamp)
        initialpose.header.frame_id = "map"
        initialpose.pose.pose.position.x = 8.2
        initialpose.pose.pose.position.y = 39.1
        initialpose.pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -2.78)
        initialpose.pose.pose.orientation.x = quat[0]
        initialpose.pose.pose.orientation.y = quat[1]
        initialpose.pose.pose.orientation.z = quat[2]
        initialpose.pose.pose.orientation.w = quat[3]
        initialpose.pose.covariance[0] = 0.01;
        initialpose.pose.covariance[7] = 0.01;
        initialpose.pose.covariance[35] = 0.01;

        # publish robot pose on initialpose topic
        for i in range(0,2):
            pub_initialpose.publish(initialpose)
            rospy.sleep(1.5)
            print "publish initpose"

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('test_name')
    if "standalone" in sys.argv:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
