#!/usr/bin/env python
import sys
import math
import rospy

from tf import transformations, TransformListener, TransformBroadcaster
from atf_recorder import RecordingManager


class PublishTf:
    def __init__(self):
        self.listener = TransformListener()
        self.br = TransformBroadcaster()
        self.pub_freq = 100.0
        self.parent_frame_id = "world"
        self.child1_frame_id = "reference1"
        self.child2_frame_id = "reference2"
        self.child3_frame_id = "reference3"
        self.child4_frame_id = "reference4"
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference2)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference3)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference4)
        rospy.sleep(1)

        recorder_0 = RecordingManager("all")
        recorder_1 = RecordingManager("test1")
        recorder_2 = RecordingManager("test2")
        recorder_3 = RecordingManager("test3")

        recorder_0.start()
        recorder_1.start()
        self.pub_line(length=-1, time=2.5)
        recorder_1.stop()
        recorder_2.start()
        self.pub_quadrat(length=2, time=10)
        recorder_2.stop()
        recorder_3.start()
        self.pub_circ(radius=2, time=10)
        recorder_3.stop()
        recorder_0.stop()

    def reference2(self, event):
        self.check_for_ctrlc()
        self.pub_tf(self.child1_frame_id, self.child2_frame_id, [1, 0, 0])

    def reference3(self, event):
        self.check_for_ctrlc()
        self.pub_tf(self.child1_frame_id, self.child3_frame_id, [math.sin(rospy.Time.now().to_sec()), 0, 0])

    def reference4(self, event):
        self.check_for_ctrlc()
        self.pub_tf(self.child1_frame_id, self.child4_frame_id, [math.sin(rospy.Time.now().to_sec()),
                                                                 math.cos(rospy.Time.now().to_sec()), 0])

    def pub_tf(self, parent_frame_id, child1_frame_id, xyz=[0, 0, 0], rpy=[0, 0, 0]):
        self.check_for_ctrlc()
        self.br.sendTransform((xyz[0], xyz[1], xyz[2]), transformations.quaternion_from_euler(
            rpy[0], rpy[1], rpy[2]), rospy.Time.now(), child1_frame_id, parent_frame_id)

    def pub_line(self, length=1, time=1):
        rospy.loginfo("Line")
        rate = rospy.Rate(self.pub_freq)

        for i in range((int(self.pub_freq*time/2)+1)):
            t = i/self.pub_freq/time*2
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*length, 0, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/2)+1)):
            t = i/self.pub_freq/time*2
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*length, 0, 0])
            rate.sleep()

    def pub_circ(self, radius=1, time=1):
        rospy.loginfo("Circ")
        rate = rospy.Rate(self.pub_freq)

        for i in range(int(self.pub_freq*time)+1):
            t = i/self.pub_freq/time
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [-radius*math.cos(2*math.pi*t)+radius,
                                                                     -radius*math.sin(2*math.pi*t),
                                                                     0])
            rate.sleep()

    def pub_quadrat(self, length=1, time=1):
        rospy.loginfo("Quadrat")
        rate = rospy.Rate(self.pub_freq)

        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*length, 0, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [length, t*length, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*length, length, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, (1-t)*length, 0])
            rate.sleep()

    @staticmethod
    def check_for_ctrlc():
        if rospy.is_shutdown():
            sys.exit()

'''
if __name__ == "__main__":
    rospy.init_node("publish_tf")
    PublishTf().start
'''
