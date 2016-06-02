#!/usr/bin/env python
import rospy

from atf_recorder.srv import RecorderCommand
from atf_msgs.msg import Trigger


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
        trigger = Trigger()
        trigger.header.stamp = rospy.Time.now()
        trigger.trigger = Trigger.ACTIVATE
        result = self.recorder_command(self.name, trigger)
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def pause(self):
        """
        Pauses the recording for the selected section.
        :return:
        """
        rospy.loginfo("recording manager: pause of testblock '%s'", self.name)
        trigger = Trigger()
        trigger.header.stamp = rospy.Time.now()
        trigger.trigger = Trigger.PAUSE
        result = self.recorder_command(self.name, trigger)
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def stop(self):
        """
        Stops the recording for the selected section.
        :return:
        """
        rospy.loginfo("recording manager: stop of testblock '%s'", self.name)
        trigger = Trigger()
        trigger.header.stamp = rospy.Time.now()
        trigger.trigger = Trigger.FINISH
        result = self.recorder_command(self.name, trigger)
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")

    def error(self):
        """
        Sends an error to the recorder.
        :return:
        """
        rospy.loginfo("recording manager: error in testblock '%s'", self.name)
        trigger = Trigger()
        trigger.header.stamp = rospy.Time.now()
        trigger.trigger = Trigger.ERROR
        result = self.recorder_command(self.name, trigger)
        if not result:
            rospy.logerr("Testblock name is not defined in test_config.yaml!")
