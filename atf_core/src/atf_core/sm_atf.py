#!/usr/bin/env python
import rospy
import smach
import smach_ros
import threading
import atf_core

from atf_msgs.msg import TestblockTrigger, TestblockStatus

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
            self.userdata.name = name
            smach.StateMachine.add('INACTIVE', Inactive(name, recorder_handle), 
                                   transitions={'start':'ACTIVE', 
                                                'error':'ERROR'})
            smach.StateMachine.add('ACTIVE', Active(name, recorder_handle), 
                                   transitions={'pause':'ERROR', #FIXME PAUSE
                                                'purge':'ERROR', #FIXME PURGE
                                                'stop':'SUCCEEDED',
                                                'error':'ERROR'})
#            smach.StateMachine.add('PAUSE', Pause(name, recorder_handle), 
#                                   transitions={'start':'ACTIVE',
#                                                'purge':'PURGE',
#                                                'stop':'succeeded',
#                                                'error':'error'})
#            smach.StateMachine.add('PURGE', Purge(name, recorder_handle), 
#                                   transitions={'start':'ACTIVE',
#                                                'pause':'PAUSE',
#                                                'stop':'succeeded',
#                                                'error':'error'})
            smach.StateMachine.add('SUCCEEDED', Stopped(name, recorder_handle), 
                                   transitions={'done':'succeeded'})
            smach.StateMachine.add('ERROR', GenericRecorderState(name, recorder_handle, TestblockStatus.ERROR), 
                                   transitions={'done':'error'})

##############
### states ###
##############
class Inactive(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['name'], outcomes=['start', 'error'])
        self.trigger = None
        self.recorder_handle = recorder_handle
        self._trigger_cond = threading.Condition()
        rospy.Subscriber("atf/trigger", TestblockTrigger, self.trigger_cb, name)

    def trigger_cb(self, msg, name):
        if msg.name == name:
            with self._trigger_cond:
                self.trigger = msg
                self._trigger_cond.notify()
        

    def execute(self, userdata):
        self.trigger = None
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.INACTIVE
        self.recorder_handle.record_status(status)

        with self._trigger_cond:
            self._trigger_cond.wait()

        if self.trigger.trigger == TestblockTrigger.START:
            outcome = 'start'
        elif self.trigger.trigger == TestblockTrigger.ERROR:
            outcome = 'error'
        else:
            rospy.logerr("%s: Invalid transition '%d' from inactive state"%(userdata.name, self.trigger.trigger))
            outcome = 'error'
        self.trigger = None
        return outcome


class Active(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['name'], output_keys=['user_result'], outcomes=['pause', 'purge', 'stop', 'error'])
        self.trigger = None
        self.recorder_handle = recorder_handle
        self._trigger_cond = threading.Condition()
        rospy.Subscriber("atf/trigger", TestblockTrigger, self.trigger_cb, name)

    def trigger_cb(self, msg, name):
        if msg.name == name:
            with self._trigger_cond:
                self.trigger = msg
                self._trigger_cond.notify()

    def execute(self, userdata):
        self.trigger = None
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.ACTIVE
        self.recorder_handle.record_status(status)

        # start to record metric topics into bag file
        self.recorder_handle.start_recording(userdata.name)

        # either trigger is already set or we wait for the next transition trigger
        if self.trigger == None:
            # wait for next transition trigger
            with self._trigger_cond:
                self._trigger_cond.wait()
        else:
            rospy.logerr("trigger is not None: %s", str(self.trigger))

        if self.trigger.trigger == TestblockTrigger.START:
            rospy.logerr("calling start, but testblock is already in active state")
            outcome = 'error'
        elif self.trigger.trigger == TestblockTrigger.PAUSE:
            outcome = 'pause'
        elif self.trigger.trigger == TestblockTrigger.PURGE:
            outcome = 'purge'
        elif self.trigger.trigger == TestblockTrigger.STOP:
            userdata.user_result = self.trigger.user_result
            outcome = 'stop'
        elif self.trigger.trigger == TestblockTrigger.ERROR:
            outcome = 'error'
        else:
            rospy.logerr("%s: Invalid transition from active state"%userdata.name)
            outcome = 'error'
        self.trigger = None
        return outcome

class Stopped(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['name', 'user_result'], outcomes=['done', 'error'])
        self.recorder_handle = recorder_handle

    def execute(self, userdata):
        self.trigger = None

        # start to record metric topics into bag file
        self.recorder_handle.stop_recording(userdata.name)

        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.SUCCEEDED
        status.user_result = userdata.user_result
        self.recorder_handle.record_status(status)

        return 'done'

class GenericRecorderState(smach.State):
    def __init__(self, name, recorder_handle, status):
        smach.State.__init__(self, input_keys=['name'], outcomes=['done'])
        self.recorder_handle = recorder_handle

    def execute(self, userdata):
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.SUCCEEDED
        self.recorder_handle.record_status(status)
        return 'done'

"""
class Pause(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['name'], outcomes=['start', 'purge', 'stop', 'error'])
        self.trigger = None
        self.recorder_handle = recorder_handle
        self._trigger_cond = threading.Condition()
        rospy.Subscriber("atf/trigger", TestblockTrigger, self.trigger_cb, name)

    def trigger_cb(self, msg, name):
        if msg.name == name:
            with self._trigger_cond:
                self.trigger = msg
                self._trigger_cond.notify()

    def execute(self, userdata):
        self.trigger = None
        with self._trigger_cond:
            self._trigger_cond.wait()

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
            rospy.logerr("%s: Invalid transition from pause state")
            outcome = 'error'
        self.trigger = None
        return outcome

class Purge(smach.State):
    def __init__(self, name, recorder_handle):
        smach.State.__init__(self, input_keys=['name'], outcomes=['start', 'pause', 'stop', 'error'])
        self.trigger = None
        self.recorder_handle = recorder_handle
        self._trigger_cond = threading.Condition()
        rospy.Subscriber("atf/trigger", TestblockTrigger, self.trigger_cb, name)

    def trigger_cb(self, msg, name):
        if msg.name == name:
            with self._trigger_cond:
                self.trigger = msg
                self._trigger_cond.notify()

    def execute(self, userdata):
        self.trigger = None 
        with self._trigger_cond:
            self._trigger_cond.wait()

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
            rospy.logerr("Invalid transition from purge state")
            outcome = 'error'
        self.trigger = None
        return outcome
"""
