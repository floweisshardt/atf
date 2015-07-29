#!/usr/bin/env python
import rospy
import time


class CalculateTime:
    def __init__(self):
        self.start_time = rospy.Time()
        self.stop_time = rospy.Time()

    def start(self):
        self.start_time = rospy.Time.from_sec(time.time())

    def stop(self):
        self.stop_time = rospy.Time.from_sec(time.time())

    @staticmethod
    def pause():
        pass

    def get_result(self):
        # return "Time: " + str(round(self.stop_time.to_sec()-self.start_time.to_sec(), 3)) + "s"
        return self.start_time.to_sec(), "time", round((self.stop_time.to_sec()-self.start_time.to_sec()), 3)
