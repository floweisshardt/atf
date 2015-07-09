#!/usr/bin/python
import rospy
import rospkg
import os
import datetime
import rosbag
from threading import Lock
from cob_benchmarking.msg import RecordingManagerData
import rostopic
from atf_msgs.msg import Trigger


class RecordingManager:
    def __init__(self):

        self.record_tf = False
        self.record_ressources = False
        self.lock = Lock()

        self.scene = ""
        self.path_rb = ""
        self.bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/init.bag", 'w')
        self.bag.close()
        self.run_once = False

        rospy.Subscriber("recording_manager/data", RecordingManagerData, self.data_callback, queue_size=1)
        rospy.Subscriber("recording_manager/ressources", RecordingManagerData, self.ressources_callback, queue_size=1)
        msg_type = rostopic.get_topic_class("/tf", blocking=True)[0]
        rospy.Subscriber("/tf", msg_type, self.tf_callback, queue_size=1)

    def __del__(self):

        os.remove(rospkg.RosPack().get_path("cob_benchmarking") + "/results/init.bag")

    def data_callback(self, msg):

        if not self.run_once:
            # Create RosBag
            self.scene = ("scene " + str(datetime.datetime.now()) + ".bag").replace(" ", "_")
            self.path_rb = rospkg.RosPack().get_path("cob_benchmarking") + "/results/" + self.scene
            self.bag = rosbag.Bag(self.path_rb, 'w')
            self.run_once = True
            self.record_tf = True

        self.lock.acquire()
        self.bag.write(msg.id.data, msg, rospy.Time.now())
        self.lock.release()

        if msg.id.data == "planning_timer" and msg.trigger.trigger == Trigger.ACTIVATE:
            self.record_ressources = True
            rospy.loginfo("Planning started")

        elif msg.id.data == "planning_timer" and msg.trigger.trigger == Trigger.PAUSE:
            self.record_ressources = False
            rospy.loginfo("Planning stopped")

        elif msg.id.data == "execution_timer" and msg.trigger.trigger == Trigger.ACTIVATE:
            self.record_tf = True

            self.lock.acquire()
            self.bag.write("trigger", msg.trigger, rospy.Time.now())
            self.lock.release()

            self.record_ressources = True
            rospy.loginfo("Execution started")

        elif (msg.id.data == "execution_timer" and msg.trigger.trigger == Trigger.FINISH)\
                or msg.id.data == "planning_error":

            self.lock.acquire()
            self.bag.write("trigger", msg.trigger, rospy.Time.now())
            self.lock.release()

            self.record_ressources = False
            self.record_tf = False
            rospy.loginfo("Execution stopped")

            self.lock.acquire()
            self.bag.close()
            self.lock.release()
            self.run_once = False

    def ressources_callback(self, msg):
        if self.record_ressources:
            self.lock.acquire()
            self.bag.write(msg.id.data, msg, rospy.Time.now())
            self.lock.release()

    def tf_callback(self, msg):
        if self.record_tf:
            self.lock.acquire()
            self.bag.write("tf", msg, rospy.Time.now())
            self.lock.release()

if __name__ == '__main__':
    rospy.init_node('recording_manager')
    rospy.loginfo("Starting 'Recording manager'...")
    RecordingManager()
    while not rospy.is_shutdown():
        rospy.spin()
