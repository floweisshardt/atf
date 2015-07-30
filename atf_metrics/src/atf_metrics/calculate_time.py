#!/usr/bin/env python
import rospy
import time


class CalculateTime:
    def __init__(self):
        self.start_time = rospy.Time()
        self.stop_time = rospy.Time()
        self.finished = False

    def start(self):
        self.start_time = rospy.Time.from_sec(time.time())

    def stop(self):
        self.stop_time = rospy.Time.from_sec(time.time())
        self.finished = True

    @staticmethod
    def pause():
        pass

    def get_result(self):
        if self.finished:
            return self.start_time.to_sec(), "time", round((self.stop_time.to_sec()-self.start_time.to_sec()), 3)
        else:
            return False
