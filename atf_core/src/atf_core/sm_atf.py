#!/usr/bin/env python
import rospy
import smach
import smach_ros
import threading

from atf_msgs.msg import TestblockTrigger, TestblockStatus

####################
### testblock SM ###
####################
class SmAtfTestblock(smach.Concurrence):

    def __init__(self, name, recorder_handle):
        smach.Concurrence.__init__(
            self,
            outcomes=['succeeded','error'],
            default_outcome='error',
            child_termination_cb = self.child_term_cb,
            outcome_cb = self.out_cb)

        with self:
            self.userdata.name = name
            self.userdata.trigger_list = []

            smach.Concurrence.add('GET_TRIGGERS', GetTriggers(name))

            sm_testblock = smach.StateMachine(outcomes=['succeeded','error'])
            sm_testblock.userdata = self.userdata

            with sm_testblock:
                smach.StateMachine.add('INACTIVE', Inactive(recorder_handle), 
                                    transitions={'start':'ACTIVE', 
                                                    'error':'ERROR'})
                smach.StateMachine.add('ACTIVE', Active(recorder_handle), 
                                    transitions={'pause':'ERROR', #FIXME PAUSE
                                                    'purge':'ERROR', #FIXME PURGE
                                                    'stop':'SUCCEEDED',
                                                    'error':'ERROR'})
    #            smach.StateMachine.add('PAUSE', Pause(recorder_handle), 
    #                                   transitions={'start':'ACTIVE',
    #                                                'purge':'PURGE',
    #                                                'stop':'succeeded',
    #                                                'error':'error'})
    #            smach.StateMachine.add('PURGE', Purge(recorder_handle), 
    #                                   transitions={'start':'ACTIVE',
    #                                                'pause':'PAUSE',
    #                                                'stop':'succeeded',
    #                                                'error':'error'})
                smach.StateMachine.add('SUCCEEDED', Stopped(recorder_handle), 
                                    transitions={'done':'succeeded'})
                smach.StateMachine.add('ERROR', GenericRecorderState(recorder_handle, TestblockStatus.ERROR), 
                                    transitions={'done':'error'})

            smach.Concurrence.add('SM_TESTBLOCK', sm_testblock)

    # gets called when ANY child state terminates
    def child_term_cb(self, outcome_map):

        # terminate all running states if BAR finished
        if outcome_map['SM_TESTBLOCK']:
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(self, outcome_map):
        if outcome_map['SM_TESTBLOCK'] == 'succeeded':
            return 'succeeded'
        else:
            return 'error'

##############
### states ###
##############
class GetTriggers(smach.State):
    def __init__(self, name):
        smach.State.__init__(self, 
                            input_keys=['name', 'trigger_list'],
                            output_keys=['trigger_list'],
                            outcomes=['done', 'error'])
        self.trigger_list = []
        rospy.Subscriber("atf/trigger", TestblockTrigger, self.trigger_cb, name)

    def trigger_cb(self, msg, name):
        if msg.name == name:
            rospy.logdebug("got new trigger %d for testblock %s", msg.trigger, msg.name)
            self.trigger_list.append(msg)

    def execute(self, userdata):
        r = rospy.Rate(100)
        while not self.preempt_requested():
            if len(self.trigger_list) > 0:
                userdata.trigger_list.append(self.trigger_list.pop(0))
            r.sleep()
        return "done"

class Inactive(smach.State):
    def __init__(self, recorder_handle):
        smach.State.__init__(self, input_keys=['name', 'trigger_list'], outcomes=['start', 'error'])
        self.recorder_handle = recorder_handle

    def execute(self, userdata):
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.INACTIVE
        self.recorder_handle.record_status(status)

        r = rospy.Rate(100)
        while len(userdata.trigger_list) == 0:
            r.sleep()
        trigger = userdata.trigger_list.pop(0)

        if trigger.trigger == TestblockTrigger.START:
            outcome = 'start'
        elif trigger.trigger == TestblockTrigger.ERROR:
            outcome = 'error'
        else:
            rospy.logerr("%s: Invalid transition '%d' from inactive state"%(userdata.name, trigger.trigger))
            outcome = 'error'
        return outcome


class Active(smach.State):
    def __init__(self, recorder_handle):
        smach.State.__init__(self, input_keys=['name', 'trigger_list'], output_keys=['user_result'], outcomes=['pause', 'purge', 'stop', 'error'])
        self.recorder_handle = recorder_handle

    def execute(self, userdata):
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = TestblockStatus.ACTIVE
        self.recorder_handle.record_status(status)

        # start to record metric topics into bag file
        self.recorder_handle.start_recording(userdata.name)

        r = rospy.Rate(100)
        while len(userdata.trigger_list) == 0:
            r.sleep() 
        trigger = userdata.trigger_list.pop(0)

        if trigger.trigger == TestblockTrigger.START:
            rospy.logerr("calling start, but testblock %s is already in active state", userdata.name)
            outcome = 'error'
        elif trigger.trigger == TestblockTrigger.PAUSE:
            outcome = 'pause'
        elif trigger.trigger == TestblockTrigger.PURGE:
            outcome = 'purge'
        elif trigger.trigger == TestblockTrigger.STOP:
            userdata.user_result = trigger.user_result
            outcome = 'stop'
        elif trigger.trigger == TestblockTrigger.ERROR:
            outcome = 'error'
        else:
            rospy.logerr("%s: Invalid transition from active state"%userdata.name)
            outcome = 'error'
        return outcome

class Stopped(smach.State):
    def __init__(self, recorder_handle):
        smach.State.__init__(self, input_keys=['name', 'user_result'], outcomes=['done', 'error'])
        self.recorder_handle = recorder_handle

    def execute(self, userdata):
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
    def __init__(self, recorder_handle, status):
        smach.State.__init__(self, input_keys=['name'], outcomes=['done'])
        self.recorder_handle = recorder_handle
        self.status = status

    def execute(self, userdata):
        # record to bag file
        status = TestblockStatus()
        status.stamp = rospy.Time.now()
        status.name = userdata.name
        status.status = self.status
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
