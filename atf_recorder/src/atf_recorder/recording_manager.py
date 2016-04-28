#!/usr/bin/env python
import rospy

from atf_recorder.srv import *
from atf_msgs.msg import *


class RecordingManager:
    def __init__(self, name=""):
        """
        Class for managing recordings.
        :param name: The name of the testblock you want to record.
        :type name: str
        :return:
        """
        if name == "":
            raise NameError("No testblock name defined!")
        self.name = name
        self.ns = "/atf/"
        rospy.loginfo("waiting for atf recorder service")
        rospy.wait_for_service(self.ns + "recorder_command")
        rospy.loginfo("found atf recorder service")
        self.recorder_command = rospy.ServiceProxy(self.ns + "recorder_command", RecorderCommand)

    def start(self):
        """
        Starts the recording for the selected section.
        :return:
        """
        rospy.loginfo("recording manager: start of testblock '%s'", self.name)
        result = self.recorder_command(self.name, Trigger(Trigger.ACTIVATE))
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def pause(self):
        """
        Pauses the recording for the selected section.
        :return:
        """
        rospy.loginfo("recording manager: pause of testblock '%s'", self.name)
        result = self.recorder_command(self.name, Trigger(Trigger.PAUSE))
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def stop(self):
        """
        Stops the recording for the selected section.
        :return:
        """
        rospy.loginfo("recording manager: stop of testblock '%s'", self.name)
        result = self.recorder_command(self.name, Trigger(Trigger.FINISH))
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def error(self):
        """
        Sends an error to the recorder.
        :return:
        """
        rospy.loginfo("recording manager: error in testblock '%s'", self.name)
        result = self.recorder_command(self.name, Trigger(Trigger.ERROR))
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")
