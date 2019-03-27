#!/usr/bin/env python
import rospy
import smach

class SmAtfTestblock(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=['succeeded','error'],
            input_keys=['config'],
            output_keys=[])

        with self:
            smach.StateMachine.add('INACTIVE', Inactive(), 
                                   transitions={'start':'ACTIVE', 
                                                'error':'error'})
            smach.StateMachine.add('ACTIVE', Active(), 
                                   transitions={'pause':'PAUSE',
                                                'purge':'PURGE',
                                                'stop':'succeeded',
                                                'error':'error'})
            smach.StateMachine.add('PAUSE', Pause(), 
                                   transitions={'start':'ACTIVE',
                                                'purge':'PURGE',
                                                'stop':'succeeded',
                                                'error':'error'})
            smach.StateMachine.add('PURGE', Purge(), 
                                   transitions={'start':'ACTIVE',
                                                'pause':'PAUSE',
                                                'stop':'succeeded',
                                                'error':'error'})

class Inactive(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['config', 'testblock'], output_keys=['config', 'testblock'], outcomes=['start', 'error'])
        # Your state initialization goes here
        print "Init Inactive"

    def execute(self, userdata):
        print "userdata.config:", userdata.config
        rospy.sleep(3)
        if True:
            return 'start'
        else:
            return 'error'


class Active(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['config', 'testblock'], outcomes=['pause', 'purge', 'stop', 'error'])
        # Your state initialization goes here
        print "Init Active"

    def execute(self, userdata):
        print "userdata.config:", userdata.config
        rospy.sleep(3)
        if True:
            return 'pause'
        else:
            return 'error'

class Pause(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['config', 'testblock'], outcomes=['start', 'purge', 'stop', 'error'])
        # Your state initialization goes here
        print "Init Paused"

    def execute(self, userdata):
        print "userdata.config:", userdata.config
        rospy.sleep(3)
        if True:
            return 'stop'
        else:
            return 'error'

class Purge(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['config', 'testblock'], outcomes=['start', 'pause', 'stop', 'error'])
        # Your state initialization goes here
        print "Init Purged"

    def execute(self, userdata):
        print "userdata.config:", userdata.config
        rospy.sleep(3)
        if True:
            return 'stop'
        else:
            return 'error'
