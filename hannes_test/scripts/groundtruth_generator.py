#!/usr/bin/env python
import rospy
import tf
import math
import signal
import sys

from rosbag import Bag
from optparse import OptionParser

class GroundtruthGenerator:
    def __init__(self, root_frame, measured_frame):
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.tf_sampling_freq = 20.0  # Hz
        self.finished = False
        self.transformations = []
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.file_path = "/home/fmw-hb/bagfiles/ekf_transforms.txt"
        self.shutdown = False

        #rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf())
        print "Init!"

    def record_tf(self):
        try:
            time = rospy.Time(0)
            time = self.listener.getLatestCommonTime(self.root_frame, self.measured_frame)
            self.listener.waitForTransform(self.root_frame,
                                           self.measured_frame,
                                           time,
                                           rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))
            (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, time)
            #coords = [trans[0],trans[1],trans[2]]
            self.broadcaster.sendTransform(trans, rot, time, "/base_link_ekf", "/map")

        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(e)
            pass

    def write(self):
        transforms = open(self.file_path, 'wa')
        transforms.write(str(self.transformations))
        print "transformations saved to ", self.file_path

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.shutdown = True
        rospy.logwarn("node closed - write to file")
        self.write()
        #sys.exit(0)


    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # signal.signal(signal.SIGINT, self.signal_handler)
            self.record_tf()
            rate.sleep()





if __name__ == '__main__':
    rospy.init_node('groundtruth_generator')
    # parser = OptionParser(usage="%prog -p or --popup", prog=os.path.basename(sys.argv[0]))
    # parser.add_option("-p", "--popup", action="store_true", dest="popup", default=False, help="Use to show popup on errors")
    #
    # (options, args) = parser.parse_args()
    gg = GroundtruthGenerator("/map", "/base_link")
    gg.run()

