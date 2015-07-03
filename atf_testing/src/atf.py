#!/usr/bin/env python

from state_machine import StateMachine

import rospy

from atf_msgs.msg import Status, Trigger

class ATF():
    def __init__(self, metrics):
        rospy.Subscriber("trigger", Trigger, self.trigger_callback)

        self.transition = None
        self.metrics = metrics
        
        self.m = StateMachine()
        self.m.add_state(Status.PURGED, self.purged_state)
        self.m.add_state(Status.ACTIVE, self.active_state)
        self.m.add_state(Status.PAUSED, self.paused_state)
        self.m.add_state(Status.FINISHED, None, end_state=1)
        self.m.add_state(Status.ERROR, None, end_state=1)
        self.m.set_start(Status.PURGED)

        self.m.run()

    def trigger_callback(self,msg):
        self.transition = msg.trigger

    def purge(self):
        for metric in self.metrics:
            metric.purge()

    def activate(self):
        for metric in self.metrics:
            metric.start()

    def pause(self):
        self.stop()

    def finish(self):
        self.stop()

    def stop(self):
        for metric in self.metrics:
            metric.stop()

    def purged_state(self):
        while not rospy.is_shutdown() and self.transition == None:
            #print "wait for trigger, current_state =", self.m.get_current_state()
            continue
        if self.transition == Trigger.PURGE:
            # is already purged
            newState = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            print "activate"
            self.activate()
            newState = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            newState = Status.ERROR
        elif self.transition == Trigger.FINISH:
            newState = Status.ERROR
        else:
            newState = Status.ERROR    
        self.transition = None
        return newState

    def active_state(self):
        while not rospy.is_shutdown() and self.transition == None:
            #print "wait for trigger, current_state =", self.m.get_current_state()
            continue
        if self.transition == Trigger.PURGE:
            print "purge"
            self.purge()
            newState = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            # is already active
            newState = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            print "pause"
            self.pause()
            newState = Status.PAUSED
        elif self.transition == Trigger.FINISH:
            print "finish"
            self.finish()
            newState = Status.FINISHED
        else:
            newState = Status.ERROR
        self.transition = None
        return newState

    def paused_state(self):
        while not rospy.is_shutdown() and self.transition == None:
            #print "wait for trigger, current_state =", self.m.get_current_state()
            continue
        if self.transition == Trigger.PURGE:
            print "purge"
            self.purge()
            newState = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            print "activate"
            self.activate()
            newState = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            # is already paused
            newState = Status.PAUSED
        elif self.transition == Trigger.FINISH:
            print "finish"
            self.finish()
            newState = Status.FINISHED
        else:
            newState = Status.ERROR
        self.transition = None
        return newState

if __name__== "__main__":
    rospy.init_node("atf")
    atf = ATF()
    atf.run()

