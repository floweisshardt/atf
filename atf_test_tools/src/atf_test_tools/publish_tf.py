#!/usr/bin/env python
import math
import sys

import numpy as np

import rospy
import tf_conversions
import tf2_ros


from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

class PublishTf:
    def __init__(self):
        self.transform_stampeds = []
        self.br = tf2_ros.transform_broadcaster.TransformBroadcaster()
        self.pub_reference1_x = rospy.Publisher("topic_reference1_x", Float64, queue_size=10)
        self.pub_reference1_y = rospy.Publisher("topic_reference1_y", Float64, queue_size=10)
        self.pub_reference1_yaw = rospy.Publisher("topic_reference1_yaw", Float64, queue_size=10)
        self.pub_reference2_x = rospy.Publisher("topic_reference2_x", Float64, queue_size=10)
        self.pub_reference2_y = rospy.Publisher("topic_reference2_y", Float64, queue_size=10)
        self.pub_reference2_yaw = rospy.Publisher("topic_reference2_yaw", Float64, queue_size=10)
        self.pub_freq = 50.0
        self.parent_frame_id = "world"
        self.child1_frame_id = "reference1"
        self.child2_frame_id = "reference2"
        self.child3_frame_id = "reference3"
        self.child4_frame_id = "reference4"
        rospy.Timer(rospy.Duration(1 / self.pub_freq), self.timer_cb)
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference2)
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference3)
        #rospy.Timer(rospy.Duration(1 / self.pub_freq), self.reference4)
        rospy.sleep(1.0)

    def timer_cb(self, event):
        #print "TIMER reference1", len(self.transform_stampeds)
        self.check_for_ctrlc()
        if len(self.transform_stampeds) == 0:
            #print "no transforms in list"           
            return

        remaining_transform_stampeds = []
        while len(self.transform_stampeds) > 0:
            transform = self.transform_stampeds.pop(0)
            if transform.header.stamp >= rospy.Time.now():
                #print "transformation in future, skipping"
                remaining_transform_stampeds.append(transform)
                continue

            #print "found transforms", "header.stamp=", transform.header.stamp.to_sec(), rospy.Time.now().to_sec()
            if transform.child_frame_id == "reference1":
                self.pub_reference1_x.publish(transform.transform.translation.x)
                self.pub_reference1_y.publish(transform.transform.translation.y)
                self.pub_reference1_yaw.publish(tf_conversions.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2])
            elif transform.child_frame_id == "reference2":
                self.pub_reference2_x.publish(transform.transform.translation.x)
                self.pub_reference2_y.publish(transform.transform.translation.y)
                self.pub_reference2_yaw.publish(tf_conversions.transformations.euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2])
            self.br.sendTransform([transform])
        self.transform_stampeds += remaining_transform_stampeds

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


    #def pub_tf(self, parent_frame_id, child1_frame_id, xyz=[0, 0, 0], rpy=[0, 0, 0]):
    def pub_tf(self, transform_stampeds):
        self.check_for_ctrlc()
        rate = rospy.Rate(int(self.pub_freq))
        start = rospy.Time.now()

        for ts in transform_stampeds:
            try:
                self.br.sendTransform([ts])
                self.pub.publish(ts.transform.translation.x)
            except rospy.ROSException:
                rospy.logdebug("could not send transform")
            
            stop = rospy.Time.now()
            if (stop-start).to_sec() > 1/self.pub_freq:
                rospy.logwarn("Publishing tf took longer than specified loop rate " + str((stop-start).to_sec()) + ", should be less than " + str(1/self.pub_freq))
            rate.sleep()

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
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = self.parent_frame_id
        ts.child_frame_id = self.child1_frame_id
        ts.transform.translation.x = 0
        ts.transform.translation.y = 0
        ts.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        self.transform_stampeds += [ts]

    def pub_circ(self, parent_frame_id="world", child_frame_id="reference1", radius=1, period_time=1, periods=1):
        now = rospy.Time.now()
        name = "circ_" + parent_frame_id + "_" + child_frame_id + "_" + str(radius) + "_" + str(period_time) + "_" + str(periods)
        rospy.loginfo(name)

        pub_list = []

        x = 0
        y = radius

        for i in range(1, int(self.pub_freq * period_time * periods) + 1):
            t = i / self.pub_freq

            yaw = 2 * math.pi * t / period_time
            precision = 4
            x = round(radius * math.sin(yaw), precision)
            y = round(radius * math.cos(yaw), precision) - radius
            #print child_frame_id, "x, y, yaw: ", x, y, yaw

            ts = TransformStamped()
            ts.header.stamp = now + rospy.Duration(t)
            ts.header.frame_id = parent_frame_id
            ts.child_frame_id = child_frame_id
            ts.transform.translation.x = x
            ts.transform.translation.y = y
            ts.transform.translation.z = 0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, -yaw)
            ts.transform.rotation.x = q[0]
            ts.transform.rotation.y = q[1]
            ts.transform.rotation.z = q[2]
            ts.transform.rotation.w = q[3]

            pub_list.append(ts)
        self.transform_stampeds += pub_list

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
