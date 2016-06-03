#!/usr/bin/env python
import rospy

from state_machine import StateMachine
from atf_msgs.msg import TestblockState, TestblockTrigger


class Testblock:
    def __init__(self, name, metrics):

        self.name = name
 #       rospy.Subscriber("/atf/" + self.name + "/trigger", TestblockTrigger, self.trigger_callback)

        self.transition = None
        self.timestamp = None
        self.metrics = metrics
        self.atf_started = False
        self.exception = None

        self.m = StateMachine(self.name)
        self.m.add_state(TestblockState.INVALID, self._purged_state)
        self.m.add_state(TestblockState.PURGED, self._purged_state)
        self.m.add_state(TestblockState.ACTIVE, self._active_state)
        self.m.add_state(TestblockState.PAUSED, self._paused_state)
        self.m.add_state(TestblockState.SUCCEEDED, self._succeeded_state, end_state=True)
        self.m.add_state(TestblockState.ERROR, self._error_state, end_state=True)
        self.m.set_start(TestblockState.INVALID)

    #def sm_exception_cb(self, e):
    #    self.exception = e

    def _run(self):
        self.atf_started = True
        self.m.run()
        while not rospy.is_shutdown() and not self.m.running:
            continue

    def _finished(self):
        return self.m.finished

    def _trigger_callback(self, msg):
        print "testblock.py: trigger_callbackmsg=\n", msg
        self.transition = msg.trigger
        self.timestamp = msg.header.stamp

    def _wait_for_transition_is_none(self):
        while not rospy.is_shutdown() and not self.transition is None:
            if self.exception != None:
                raise TestblockError(self.exception)
            continue

    def get_state(self):
        return self.m.get_current_state()

###############
# transitions #
###############
    # purge
    def purge(self):
        self._wait_for_transition_is_none()
        if not self.atf_started:
            raise TestblockError("Calling purge for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise TestblockError("Calling purge for testblock '%s' while testblock already stopped." % self.name)
        self.transition = TestblockTrigger.PURGE
    
    def _purge(self):
        rospy.loginfo("Purging testblock '%s'", self.name)
        for metric in self.metrics:
            metric.purge(self.timestamp)

    # start
    def start(self):
        self._wait_for_transition_is_none()
        if not self.atf_started:
            raise TestblockError("Calling start for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise TestblockError("Calling start for testblock '%s' while testblock already stopped." % self.name)
        self.transition = TestblockTrigger.START

    def _start(self):
        rospy.loginfo("Starting testblock '%s'", self.name)
        for metric in self.metrics:
            metric.start(self.timestamp)

    # pause
    def pause(self):
        self._wait_for_transition_is_none()
        if not self.atf_started:
            raise TestblockError("Calling pause for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise TestblockError("Calling pause for testblock '%s' while testblock already stopped." % self.name)
        self.transition = TestblockTrigger.PAUSE
    
    def _pause(self):
        rospy.loginfo("Pausing testblock '%s'", self.name)
        for metric in self.metrics:
            metric.pause(self.timestamp)

    # stop
    def stop(self):
        self._wait_for_transition_is_none()
        if not self.atf_started:
            raise TestblockError("Calling stop for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise TestblockError("Calling stop for testblock '%s' while testblock already stopped." % self.name)
        self.transition = TestblockTrigger.STOP
    
    def _stop(self):
        rospy.loginfo("Stopping testblock '%s'", self.name)
        for metric in self.metrics:
            metric.stop(self.timestamp)

##########
# states #
##########
    def _purged_state(self):
        if self.transition == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.transition == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.name, str(self.transition), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise TestblockError(message)
        self.transition = None
        return new_state

    def _active_state(self):
        if self.transition == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.transition == TestblockTrigger.PAUSE:
            self._pause()
            new_state = TestblockState.PAUSED
        elif self.transition == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.name, str(self.transition), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ACTIVE
            self.exception = message
            raise TestblockError(message)
        self.transition = None
        return new_state

    def _paused_state(self):
        if self.transition == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.transition == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.transition == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.name, str(self.transition), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise TestblockError(message)
        self.transition = None
        return new_state

    def _succeeded_state(self):
        pass

    def _error_state(self):
        pass

class TestblockError(Exception):
    pass
