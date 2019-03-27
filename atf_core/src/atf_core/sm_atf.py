#!/usr/bin/env python
import rospy
import smach
import smach_ros
from atf_core.atf import ATFError
import atf_core

from atf_msgs.msg import TestblockTrigger

####################
### testblock SM ###
####################
class SmAtfTestblock(smach.StateMachine):

    def __init__(self, name, recorder_handle):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded','error'],
            input_keys=[],
            output_keys=[])

        with self:
            smach.StateMachine.add('INACTIVE', Inactive(name, recorder_handle), 
                                   transitions={'start':'ACTIVE', 
                                                'error':'error'})
            smach.StateMachine.add('ACTIVE', Active(name), 
                                   transitions={'pause':'PAUSE',
                                                'purge':'PURGE',
                                                'stop':'succeeded',
                                                'error':'error'})
            smach.StateMachine.add('PAUSE', Pause(name), 
                                   transitions={'start':'ACTIVE',
                                                'purge':'PURGE',
                                                'stop':'succeeded',
                                                'error':'error'})
            smach.StateMachine.add('PURGE', Purge(name), 
                                   transitions={'start':'ACTIVE',
                                                'pause':'PAUSE',
                                                'stop':'succeeded',
                                                'error':'error'})

##############
### states ###
##############
class Inactive(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['recorder_handle'], outcomes=['start', 'error'])
        rospy.Subscriber("atf/" + name + "/trigger", TestblockTrigger, self.trigger_cb)
        self.trigger = None
        self.recorder_handle = recorder_handle

    def trigger_cb(self, msg):
        # record to bag file
        #userdata.recorder_handle.record_trigger(self.trigger)
        self.recorder_handle.record_trigger(msg)
        self.trigger = msg

    def execute(self, userdata):
        self.trigger = None
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.trigger == None:
                r.sleep()
                continue
            if self.trigger.trigger == TestblockTrigger.START:
                outcome = 'start'
            else:
                outcome = 'error'
            self.trigger = None
            return outcome


class Active(smach.State):
    def __init__(self, name):
        smach.State.__init__(self, outcomes=['pause', 'purge', 'stop', 'error'])
        rospy.Subscriber("atf/" + name + "/trigger", TestblockTrigger, self.trigger_cb)
        self.trigger = None

    def trigger_cb(self, msg):
        self.trigger = msg.trigger

    def execute(self, userdata):
        self.trigger = None
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.trigger == None:
                # TODO record topics for testblock to bag file
                r.sleep()
                continue
            if self.trigger == TestblockTrigger.START:
                rospy.logerr("calling start, but testblock is already in active state")
                outcome = 'error'
            elif self.trigger == TestblockTrigger.PAUSE:
                outcome = 'pause'
            elif self.trigger == TestblockTrigger.PURGE:
                outcome = 'purge'
            elif self.trigger == TestblockTrigger.STOP:
                outcome = 'stop'
            else:
                outcome = 'error'
            self.trigger = None
            return outcome

class Pause(smach.State):
    def __init__(self, name):
        smach.State.__init__(self, outcomes=['start', 'purge', 'stop', 'error'])
        rospy.Subscriber("atf/" + name + "/trigger", TestblockTrigger, self.trigger_cb)
        self.trigger = None

    def trigger_cb(self, msg):
        self.trigger = msg.trigger

    def execute(self, userdata):
        self.trigger = None
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.trigger == None:
                r.sleep()
                continue
            if self.trigger == TestblockTrigger.START:
                outcome = 'start'
            elif self.trigger == TestblockTrigger.PAUSE:
                rospy.logerr("calling pause, but testblock is already in pause state")
                outcome = 'error'
            elif self.trigger == TestblockTrigger.PURGE:
                outcome = 'purge'
            elif self.trigger == TestblockTrigger.STOP:
                outcome = 'stop'
            else:
                outcome = 'error'
            self.trigger = None
            return outcome

class Purge(smach.State):
    def __init__(self, name):
        smach.State.__init__(self, outcomes=['start', 'pause', 'stop', 'error'])
        rospy.Subscriber("atf/" + name + "/trigger", TestblockTrigger, self.trigger_cb)
        self.trigger = None

    def trigger_cb(self, msg):
        self.trigger = msg.trigger

    def execute(self, userdata):
        self.trigger = None
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.trigger == None:
                r.sleep()
                continue
            if self.trigger == TestblockTrigger.START:
                outcome = 'start'
            elif self.trigger == TestblockTrigger.PAUSE:
                outcome = 'pause'
            elif self.trigger == TestblockTrigger.PURGE:
                rospy.logerr("calling purge, but testblock is already in purge state")
                outcome = 'error'
            elif self.trigger == TestblockTrigger.STOP:
                outcome = 'stop'
            else:
                outcome = 'error'
            self.trigger = None
            return outcome
