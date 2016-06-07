#!/usr/bin/env python
import rospy

from atf_core import StateMachine#, ATFRecorder
from atf_msgs.msg import TestblockState, TestblockTrigger


class Testblock:
    def __init__(self, name, metrics, recorder_handle):

        self.name = name
        self.metrics = metrics
        self.recorder_handle = recorder_handle
        self.trigger = None
        self.timestamp = None
        self.exception = None
        self.atf_started = False

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

    def _wait_for_transition_is_done(self):
        while not rospy.is_shutdown() and not self.trigger is None:
            if self.exception != None:
                raise ATFTestblockError(self.exception)
            continue

    def _wait_while_transition_is_active(self):
        while not rospy.is_shutdown() and self.trigger is None:
            if self.exception != None:
                raise ATFTestblockError(self.exception)
            continue

    def get_state(self):
        return self.m.get_current_state()

###############
# transitions #
###############
    # purge
    def purge(self):
        # wait for old transition to finish before processing new one
        self._wait_for_transition_is_done()

        if not self.atf_started:
            raise ATFTestblockError("Calling purge for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise ATFTestblockError("Calling purge for testblock '%s' while testblock already stopped." % self.name)

        # set new transition trigger
        t = TestblockTrigger()
        t.stamp = rospy.Time.now()
        t.name = self.name
        t.trigger = TestblockTrigger.PURGE
        self.trigger = t

    def _purge(self):
        rospy.loginfo("Purging testblock '%s'", self.name)
        for metric in self.metrics:
            metric.purge(self.timestamp)

    # start
    def start(self):
        # wait for old transition to finish before processing new one
        self._wait_for_transition_is_done()

        if not self.atf_started:
            raise ATFTestblockError("Calling start for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise ATFTestblockError("Calling start for testblock '%s' while testblock already stopped." % self.name)

        # set new transition trigger
        t = TestblockTrigger()
        t.stamp = rospy.Time.now()
        t.name = self.name
        t.trigger = TestblockTrigger.START
        self.trigger = t

    def _start(self):
        rospy.loginfo("Starting testblock '%s'", self.name)
        for metric in self.metrics:
            #print "testblock: start metric", metric # TODO: what needs to be done with metric in case of recording/analysing???
            metric.start(self.timestamp)

    # pause
    def pause(self):
        # wait for old transition to finish before processing new one
        self._wait_for_transition_is_done()

        if not self.atf_started:
            raise ATFTestblockError("Calling pause for testblock '%s' before ATF has been started." % self.name)
        if self.get_state() in self.m.endStates:
            raise ATFTestblockError("Calling pause for testblock '%s' while testblock already stopped." % self.name)

        # set new transition trigger
        t = TestblockTrigger()
        t.stamp = rospy.Time.now()
        t.name = self.name
        t.trigger = TestblockTrigger.PAUSE
        self.trigger = t

    def _pause(self):
        rospy.loginfo("Pausing testblock '%s'", self.name)
        for metric in self.metrics:
            metric.pause(self.timestamp)

    # stop
    def stop(self):
        # wait for old transition to finish before processing new one
        self._wait_for_transition_is_done()

        if not self.atf_started:
            raise ATFTestblockError("Calling stop for testblock '%s' before ATF has been started." % self.name)
        state = self.get_state()
        if state == TestblockState.INVALID:
            raise ATFTestblockError("Calling stop for testblock '%s' before testblock has been started." % self.name)
        if state in self.m.endStates:
            raise ATFTestblockError("Calling stop for testblock '%s' while testblock already in an end state." % self.name)

        # set new transition trigger
        t = TestblockTrigger()
        t.stamp = rospy.Time.now()
        t.name = self.name
        t.trigger = TestblockTrigger.STOP
        self.trigger = t

    def _stop(self):
        rospy.loginfo("Stopping testblock '%s'", self.name)
        for metric in self.metrics:
            metric.stop(self.timestamp)

##########
# states #
##########
    def _purged_state(self):
        self._wait_while_transition_is_active()
        self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.trigger.trigger == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise ATFTestblockError(message)
        self.trigger = None
        return new_state

    def _active_state(self):
        self._wait_while_transition_is_active()
        self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.trigger.trigger == TestblockTrigger.PAUSE:
            self._pause()
            new_state = TestblockState.PAUSED
        elif self.trigger.trigger == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ACTIVE
            self.exception = message
            raise ATFTestblockError(message)
        self.trigger = None
        return new_state

    def _paused_state(self):
        self._wait_while_transition_is_active()
        self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.trigger.trigger == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.trigger.trigger == TestblockTrigger.STOP:
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise ATFTestblockError(message)
        self.trigger = None
        return new_state

    def _succeeded_state(self): # will never be called
        self._wait_while_transition_is_active()

    def _error_state(self): # will never be called
        self._wait_while_transition_is_active()

class ATFTestblockError(Exception):
    pass
