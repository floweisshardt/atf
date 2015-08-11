#!/usr/bin/env python
import rospy

from atf_recorder.srv import *
from atf_msgs.msg import *


class RecordingManager:
    def __init__(self, name):

        self.name = name
        self.topic = "/testing/"
        rospy.wait_for_service(self.topic + "recorder_command")
        self.recorder_command = rospy.ServiceProxy(self.topic + "recorder_command", RecorderCommand)

    def start(self):

        # rospy.loginfo("Section '" + self.name + "': Start")

        result = self.recorder_command(self.name, Trigger(Trigger.ACTIVATE))
        if not result:
            rospy.logerr("Invalid command")

    def pause(self):

        # rospy.loginfo("Section '" + self.name + "': Pause")

        result = self.recorder_command(self.name, Trigger(Trigger.PAUSE))
        if not result:
            rospy.logerr("Invalid command")

    def stop(self):

        # rospy.loginfo("Section '" + self.name + "': Stop")

        result = self.recorder_command(self.name, Trigger(Trigger.FINISH))
        if not result:
            rospy.logerr("Invalid command")

    def error(self):

        # rospy.loginfo("Section '" + self.name + "': Error")

        result = self.recorder_command(self.name, Trigger(Trigger.ERROR))
        if not result:
            rospy.logerr("Invalid command")

    def __del__(self):
        self.recorder_command.close()
