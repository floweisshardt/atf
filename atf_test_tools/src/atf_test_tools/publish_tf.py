#!/usr/bin/env python
import math
import sys

import numpy as np

import rospy
from tf import transformations, TransformBroadcaster


class PublishTf:
    def __init__(self):
        self.br = TransformBroadcaster()
        self.pub_freq = 20.0
        self.parent_frame_id = "world"
        self.child1_frame_id = "reference1"
        self.child2_frame_id = "reference2"
        self.child3_frame_id = "reference3"
        self.child4_frame_id = "reference4"
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference2)
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference3)
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference4)
        rospy.sleep(1.0)

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

    @staticmethod
    def rotate(x, y, angle, precision=4):
        xr = round(x * math.cos(angle) - y * math.sin(angle), precision)
        yr = round(x * math.sin(angle) + y * math.cos(angle), precision)
        return xr, yr


    def pub_tf(self, parent_frame_id, child1_frame_id, xyz=[0, 0, 0], rpy=[0, 0, 0]):
        self.check_for_ctrlc()
        start = rospy.Time.now()
        try:
            self.br.sendTransform((xyz[0], xyz[1], xyz[2]), transformations.quaternion_from_euler(
                rpy[0], rpy[1], rpy[2]), rospy.Time.now(), child1_frame_id, parent_frame_id)
        except rospy.ROSException:
            rospy.logdebug("could not send transform")
        stop = rospy.Time.now()
        if (stop-start).to_sec() > 1/self.pub_freq:
            rospy.logwarn("Publishing tf took longer than specified loop rate " + str((stop-start).to_sec()) + ", should be less than " + str(1/self.pub_freq))

    def pub_line(self, length=1, time=1, same_start_stop_orientation=True):
        rospy.loginfo("Line")
        rate = rospy.Rate(int(self.pub_freq))

        n_half = int(self.pub_freq * time / 2) + 1
        for i in range(n_half):
            t = i / self.pub_freq / time * 2
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [t * length, 0, 0])
            rate.sleep()

        for i in range(n_half):
            is_last = i == n_half - 1
            t = i / self.pub_freq / time * 2
            a = math.pi if not is_last and same_start_stop_orientation else 0
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1 - t) * length, 0, 0], [0, 0, a])
            rate.sleep()

    def pub_zero(self, doSleep=False):
        self.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, 0, 0], [0, 0, 0])
        if doSleep:
            rate = rospy.Rate(int(self.pub_freq))
            rate.sleep()

    def pub_circ(self, radius=1, time=1):
        rospy.loginfo("Circ")
        rate = rospy.Rate(int(self.pub_freq))

        for i in range(int(self.pub_freq * time) + 1):
            t = i / self.pub_freq / time

            a = 2 * math.pi * t
            x, y = PublishTf.rotate(0, -radius, a)
            y += radius

            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [x, y, 0], [0, 0, a])
            rate.sleep()

    def pub_quadrat(self, length=1, time=1, same_start_stop_orientation=True):
        rospy.loginfo("Quadrat")
        rate = rospy.Rate(int(self.pub_freq))

        distance = np.linspace(0, length * 4, num=int(self.pub_freq*time) + 1)

        xy = np.vstack((distance, np.zeros_like(distance)))
        a = np.zeros_like(distance)

        alpha = math.pi / 2
        r = np.round(
            np.array([
                [np.cos(alpha), -math.sin(alpha)],
                [math.sin(alpha), math.cos(alpha)],
            ]),
            6
        )

        sidx = np.argmax(distance > length)
        xy[:, sidx:] = r.dot(xy[:, sidx:])
        xy[0, sidx:] += length
        xy[1, sidx:] -= length
        a[sidx:] += alpha

        sidx = np.argmax(distance > length * 2)
        xy[:, sidx:] = r.dot(xy[:, sidx:])
        xy[0, sidx:] += length*2
        a[sidx:] += alpha

        sidx = np.argmax(distance > length * 3)
        xy[:, sidx:] = r.dot(xy[:, sidx:])
        xy[0, sidx:] += length
        xy[1, sidx:] += length
        a[sidx:] += alpha

        if same_start_stop_orientation:
            a[-1] = a[0]

        xya = np.vstack((xy, a))

        for x, y, a in xya.T:
            self.pub_tf(self.parent_frame_id, self.child1_frame_id, [float(x), float(y), 0], [0, 0, float(a)])
            rate.sleep()



    def check_for_ctrlc(self):
        if rospy.is_shutdown():
            sys.exit()
