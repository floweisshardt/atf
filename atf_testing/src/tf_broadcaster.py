#!/usr/bin/env python

import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped
from atf_msgs.msg import Trigger

class PublishTf():
    def __init__(self, *args):
        rospy.init_node("broadcaster")
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.trigger = rospy.Publisher('trigger', Trigger)
        self.pub_freq = 100.0
        self.parent_frame_id = "base_laser_front_link"
        self.child1_frame_id = "reference1"
        self.child2_frame_id = "reference2"
        self.child3_frame_id = "reference3"
        self.child4_frame_id = "reference4"
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference2)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference3)
        rospy.Timer(rospy.Duration(1/self.pub_freq), self.reference4)
        rospy.sleep(1)

    def pub_trigger(self, trigger):
        self.trigger.publish(trigger)

    def reference2(self, event):
        self.pub_tf(self.child1_frame_id, self.child2_frame_id, [1, 0, 0])

    def reference3(self, event):
        self.pub_tf(self.child1_frame_id, self.child3_frame_id, [math.sin(rospy.Time.now().to_sec()), 0, 0])

    def reference4(self, event):
        self.pub_tf(self.child1_frame_id, self.child4_frame_id, [math.sin(rospy.Time.now().to_sec()), math.cos(rospy.Time.now().to_sec()), 0])

    def pub_tf(self, parent_frame_id, child1_frame_id, xyz = [0, 0, 0], rpy = [0, 0, 0]):
        self.br.sendTransform((xyz[0], xyz[1], xyz[2]),
                     tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
                     rospy.Time.now(),
                     child1_frame_id,
                     parent_frame_id)

    def pub_line(self, length = 1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start", rospy.Time.now().to_sec()
        for i in range((int(self.pub_freq*time/2)+1)):
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*length, 0, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/2)+1)):
            t = i/self.pub_freq/time*2
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*length, 0, 0])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()
                     
    def pub_circ(self, radius = 1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start", rospy.Time.now().to_sec()
        for i in range(int(self.pub_freq*time)+1):
            t = i/self.pub_freq/time
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [-radius*math.cos(2*math.pi*t)+radius, -radius*math.sin(2*math.pi*t), 0])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()

    def pub_quadrat(self, length = 1, time = 1):
        rate = rospy.Rate(self.pub_freq)
        print "start", rospy.Time.now().to_sec()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [t*length, 0, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [length, t*length, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [(1-t)*length, length, 0])
            rate.sleep()
        for i in range((int(self.pub_freq*time/4)+1)):
            t = i/self.pub_freq/time*4
            PTf.pub_tf(self.parent_frame_id, self.child1_frame_id, [0, (1-t)*length, 0])
            rate.sleep()
        print "end", rospy.Time.now().to_sec()

if __name__ == '__main__':
    PTf = PublishTf()
    
#    while not rospy.is_shutdown():
    PTf.pub_line(length = -1, time = 2.5)
    
    PTf.pub_trigger(Trigger(Trigger.ACTIVATE))
    PTf.pub_quadrat(length = 2, time = 10)
    PTf.pub_trigger(Trigger(Trigger.PAUSE))
    
    PTf.pub_circ(radius = 2, time = 10)

    PTf.pub_trigger(Trigger(Trigger.FINISH))
