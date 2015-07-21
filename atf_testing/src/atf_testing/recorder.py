#!/usr/bin/env python

import rospy
import rosbag
import rostopic


from tf.msg import tfMessage
from atf_msgs.msg import Trigger


class RecordTopic():
	
    def __init__(self, name, bag):
        self.name = name
        self.type = rostopic.get_topic_class(self.name, blocking=True)[0]
        self.bag = bag
        self.active = False
        rospy.Subscriber(self.name, self.type, self.callback)

    def set_active(self):
        self.active = True

    def set_pause(self):
        self.active = False

    def callback(self, msg):
        if self.active:
            self.bag.write(self.name, msg)


class ATF_Recorder():
    def __init__(self, topics):
        self.topics = []
        self.bag = rosbag.Bag('test1.bag', 'w')
        self.prepare(topics)

    def prepare(self, topics):
        for topic in topics:
            self.topics.append(RecordTopic(topic, self.bag))

    def set_active(self):
        for topic in self.topics:
            topic.set_active()

    def set_pause(self):
        for topic in self.topics:
            topic.set_pause()

    def activate_recording(self):
        self.set_active()
        t = Trigger()
        t.trigger = Trigger.ACTIVATE
        self.bag.write('trigger', t)

    def pause_recording(self):
        self.set_pause()
        t = Trigger()
        t.trigger = Trigger.PAUSE
        self.bag.write('trigger', t)

    def finish_recording(self):
        self.set_pause()
        t = Trigger()
        t.trigger = Trigger.FINISH
        self.bag.write('trigger', t)
        self.bag.close()
