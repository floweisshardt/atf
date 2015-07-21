#!/usr/bin/env python
from state_machine import StateMachine
import rospy

from atf_msgs.msg import Status, Trigger


class ATF:
    def __init__(self, metrics):
        rospy.Subscriber("/testing/execution_2/Trigger", Trigger, self.trigger_callback)

        self.transition = None
        self.metrics = metrics
        
        self.m = StateMachine()
        self.m.add_state(Status.PURGED, self.purged_state)
        self.m.add_state(Status.ACTIVE, self.active_state)
        self.m.add_state(Status.PAUSED, self.paused_state)
        self.m.add_state(Status.FINISHED, self.finished_state, end_state=True)
        self.m.add_state(Status.ERROR, self.error_state, end_state=True)
        self.m.set_start(Status.PURGED)

        self.m.run()

    def trigger_callback(self, msg):
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
    
    def get_state(self):
        return self.m.get_current_state()

    def purged_state(self):
        while not rospy.is_shutdown() and self.transition is None:
            continue

        if self.transition == Trigger.PURGE:
            # is already purged
            new_state = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            print "activate"
            self.activate()
            new_state = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            new_state = Status.ERROR
        elif self.transition == Trigger.FINISH:
            new_state = Status.ERROR
        else:
            new_state = Status.ERROR
        self.transition = None
        return new_state

    def active_state(self):
        while not rospy.is_shutdown() and self.transition is None:
            continue
        if self.transition == Trigger.PURGE:
            print "purge"
            self.purge()
            new_state = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            # is already active
            new_state = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            print "pause"
            self.pause()
            new_state = Status.PAUSED
        elif self.transition == Trigger.FINISH:
            print "finish"
            self.finish()
            new_state = Status.FINISHED
        else:
            new_state = Status.ERROR
        self.transition = None
        return new_state

    def paused_state(self):
        while not rospy.is_shutdown() and self.transition is None:
            continue
        if self.transition == Trigger.PURGE:
            print "purge"
            self.purge()
            new_state = Status.PURGED
        elif self.transition == Trigger.ACTIVATE:
            print "activate"
            self.activate()
            new_state = Status.ACTIVE
        elif self.transition == Trigger.PAUSE:
            # is already paused
            new_state = Status.PAUSED
        elif self.transition == Trigger.FINISH:
            print "finish"
            self.finish()
            new_state = Status.FINISHED
        else:
            new_state = Status.ERROR
        self.transition = None
        return new_state

    def finished_state(self):
        pass

    def error_state(self):
        pass
