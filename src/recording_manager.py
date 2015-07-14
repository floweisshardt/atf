#!/usr/bin/env python
import rospy
import rosbag
import rospkg
import rosparam
import time

from cob_benchmarking.msg import Recorder
from threading import Lock
from atf_msgs.msg import *

bag = rosbag.Bag(rospkg.RosPack().get_path("cob_benchmarking") + "/results/" + rosparam.get_param("/test_name") +
                 "_trigger.bag", 'w')
lock = Lock()


class RecordingManager:
    def __init__(self, name):

        self.name = name

        self.topic = "/testing/" + name + "/"
        self.pub_recorder_commands = rospy.Publisher(self.topic + "recorder_commands", Recorder, queue_size=1)

        print self.name + ": init"

    def start(self):
        global bag, lock
        print self.name + ": start"

        time_msg = Time()
        trigger_msg = Trigger()
        recorder_command = Recorder()

        time_msg.timestamp = rospy.Time.from_sec(time.time())
        trigger_msg.trigger.trigger = Trigger.ACTIVATE
        recorder_command.name = self.name
        recorder_command.timestamp = time_msg.timestamp
        recorder_command.trigger = trigger_msg.trigger.trigger

        self.pub_recorder_commands.publish(recorder_command)

        lock.acquire()
        bag.write(self.topic + "Timer", time_msg, rospy.Time.from_sec(time.time()))
        bag.write(self.topic + "Trigger", trigger_msg, rospy.Time.from_sec(time.time()))
        lock.release()

    def stop(self):
        global bag, lock
        print self.name + ": stop"

        time_msg = Time()
        trigger_msg = Trigger()
        recorder_command = Recorder()

        time_msg.timestamp = rospy.Time.from_sec(time.time())
        trigger_msg.trigger.trigger = Trigger.FINISH
        recorder_command.name = self.name
        recorder_command.timestamp = time_msg.timestamp
        recorder_command.trigger = trigger_msg.trigger.trigger

        self.pub_recorder_commands.publish(recorder_command)

        lock.acquire()
        bag.write(self.topic + "Timer", time_msg, rospy.Time.from_sec(time.time()))
        bag.write(self.topic + "Trigger", trigger_msg, rospy.Time.from_sec(time.time()))
        lock.release()

    def __del__(self):
        global bag
        bag.close()
