#!/usr/bin/env python
import rospy
import time

from cob_benchmarking.msg import Recorder
from atf_msgs.msg import *


class RecordingManager:
    def __init__(self, name):

        self.name = name
        self.topic = "/testing/"
        self.pub_recorder_commands = rospy.Publisher(self.topic + "recorder_commands", Recorder, queue_size=1)

    def start(self):

        rospy.loginfo(self.name + ": Start")

        recorder_command = Recorder()
        now = rospy.Time.from_sec(time.time())

        recorder_command.name = self.name
        recorder_command.timestamp = now
        recorder_command.trigger = Trigger(Trigger.ACTIVATE)

        self.pub_recorder_commands.publish(recorder_command)

    def stop(self):

        rospy.loginfo(self.name + ": Stop")

        recorder_command = Recorder()
        now = rospy.Time.from_sec(time.time())

        recorder_command.name = self.name
        recorder_command.timestamp = now
        recorder_command.trigger = Trigger(Trigger.FINISH)

        self.pub_recorder_commands.publish(recorder_command)

    def __del__(self):
        self.pub_recorder_commands.unregister()
