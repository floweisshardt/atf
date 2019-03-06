#!/usr/bin/env python
import os
import rospy
import shutil

from atf_core import StateMachine#, ATFRecorder
from atf_msgs.msg import TestblockState, TestblockTrigger

class Testblock:
    def __init__(self, name, metric_handles, recorder_handle):

        self.name = name
        self.metric_handles = metric_handles
        self.recorder_handle = recorder_handle
        self.trigger = None
        self.timestamp = None
        self.exception = None
        self.atf_started = False
        self.state = None

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
        rospy.logdebug("-------------> ENTRY  wait_for_transition_is_done ")
        rospy.logdebug(" waiting for trasition with trigger value : '%s'", self.trigger)
        while not rospy.is_shutdown() and not self.trigger is None:
            if self.exception != None:
                raise ATFTestblockError(self.exception)
            continue
        rospy.logdebug("-------------> EXIT  wait_for_transition_is_done ")

    def _wait_while_transition_is_active(self):
        rospy.logdebug("========> ENTRY  _wait_while_transition_is_active ")
        rospy.logdebug(" waiting for active transition with trigger value : '%s'", self.trigger)
        while not rospy.is_shutdown() and self.trigger is None:
            if self.exception != None:
                raise ATFTestblockError(self.exception)
            continue
        rospy.logdebug("  >>>>>>>>>>>>>>>> triggering the recording with trigger value : '%s'", self.trigger.trigger)
        if self.trigger is not None:
            self.recorder_handle.record_trigger(self.trigger)
            rospy.logdebug("  <<<<<<<<<<<<<<<<< recording has been triggered with trigger value : '%s'", self.trigger.trigger)
        rospy.logdebug("========> EXIT  _wait_while_transition_is_active ")

    #def get_state(self):
    #       return self.m.get_current_state()

    def get_result(self):
        result = {}
        #overall_groundtruth_result = None
        #overall_groundtruth_error_message = "groundtruth missmatch for: "

        if self.state == TestblockState.ERROR:
            rospy.logerr("An error occured during analysis of testblock '%s', no useful results available.")
            result.update({name: {"status": "error"}})
        else:
            #print "testblock.metrics=", testblock.metrics
            for metric_handle in self.metric_handles:
                #print "metric_handle=", metric_handle
                metric_result = metric_handle.get_result()
                #print "metric_result=", metric_result
                if metric_result is not False:
                    (metric_name, data, groundtruth_result, groundtruth, groundtruth_epsilon, details) = metric_result
                    if metric_name not in result:
                        result[metric_name] = []
                    result[metric_name].append({"data":data, "groundtruth_result": groundtruth_result, "groundtruth": groundtruth, "groundtruth_epsilon": groundtruth_epsilon, "details": details})
                    if groundtruth_result == None:
                        pass
                    elif not groundtruth_result:
                        overall_groundtruth_result = False
                        overall_groundtruth_error_message += self.name + "(" + metric_name + ": data=" + str(data) + ", groundtruth=" + str(groundtruth) + "+-" + str(groundtruth_epsilon) + " details:" + str(details) + "); "
                else:
                    raise ATFAnalyserError("No result for metric '%s' in testblock '%s'" % (metric_name, self.name))

        #if result == {}:
        #    raise ATFAnalyserError("Analysing failed, no result available.")
        #return overall_groundtruth_result, overall_groundtruth_error_message, result
        return result

###############
# transitions #
###############
    # purge
    def purge(self):
        rospy.logdebug("### ENTRY  purge  ###")
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
        rospy.logdebug("### EXIT purge  ###")

    def _purge(self):
        rospy.loginfo("Purging testblock '%s'", self.name)
        for metric_handle in self.metric_handles:
            metric_handle.purge(self.timestamp)

    # start
    def start(self):
        rospy.loginfo("### ENTRY  start  ###")
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
        rospy.logdebug(" start call with trigger : '%s'", self.trigger.trigger)
        rospy.logdebug("### EXIT  start  ###")

    def _start(self):
        rospy.loginfo("Starting testblock '%s'", self.name)
        for metric_handle in self.metric_handles:
            #print "testblock: start metric", metric # TODO: what needs to be done with metric in case of recording/analysing???
            metric_handle.start(self.timestamp)

    # pause
    def pause(self):
        rospy.logdebug("### ENTRY  pause  ###")
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
        rospy.logdebug("### EXIT  pause  ###")

    def _pause(self):
        rospy.loginfo("Pausing testblock '%s'", self.name)
        for metric_handle in self.metric_handles:
            metric_handle.pause(self.timestamp)

    # stop
    def stop(self):
        rospy.logdebug("### ENTRY  stop  ###")
        self.stop_call = "testblock initiated stop call"
        # wait for old transition to finish before processing new one
        rospy.logdebug(" Calling - _wait_for_transition_is_done() ***")
        self._wait_for_transition_is_done()
        rospy.logdebug(" Return - _wait_for_transition_is_done() ***")

        if not self.atf_started:
            raise ATFTestblockError("Calling stop for testblock '%s' before ATF has been started." % self.name)
        state = self.get_state()
        if state == TestblockState.INVALID:
            raise ATFTestblockError("Calling stop for testblock '%s' before testblock has been started." % self.name)
        if state in self.m.endStates:
            raise ATFTestblockError("Calling stop for testblock '%s' while testblock already in an end state." % self.name)

        rospy.logdebug("  Setting new trigger STOP ***")
        # set new transition trigger
        t = TestblockTrigger()
        t.stamp = rospy.Time.now()
        t.name = self.name
        t.trigger = TestblockTrigger.STOP
        self.trigger = t
        rospy.logdebug("### EXIT  stop  ###")

    def _stop(self):
        rospy.loginfo("Stopping testblock '%s'", self.name)
        for metric_handle in self.metric_handles:
            metric_handle.stop(self.timestamp)

##########
# states #
##########
    def _purged_state(self):
        rospy.logdebug("*** ENTRY _purged_state ***")
        self._wait_while_transition_is_active()
        #self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.trigger.trigger == TestblockTrigger.STOP:
            rospy.logdebug("Stopping testblock is called from _purged_state")
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise ATFTestblockError(message)
        rospy.logdebug(" _purged_state trigger : '%s'", self.trigger.trigger)
        self.trigger = None
        rospy.logdebug(" _purged_state after trigger = None : '%s'", self.trigger)
        rospy.logdebug("*** EXIT _purged_state ***")
        return new_state

    def _active_state(self):
        rospy.logdebug("*** ENTRY _active_state ***")
        self._wait_while_transition_is_active()
        #self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.trigger.trigger == TestblockTrigger.PAUSE:
            self._pause()
            new_state = TestblockState.PAUSED
        elif self.trigger.trigger == TestblockTrigger.STOP:
            rospy.logdebug("Stopping testblock is called from _active_state")
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ACTIVE
            self.exception = message
            raise ATFTestblockError(message)
        rospy.logdebug(" _active_state trigger - _active_state : '%s'", self.trigger.trigger)
        self.trigger = None
        rospy.logdebug(" _active_state after trigger = None : '%s'", self.trigger)
        rospy.logdebug("*** EXIT _active_state ***")
        return new_state

    def _paused_state(self):
        rospy.logdebug("*** ENTRY _paused_state ***")
        self._wait_while_transition_is_active()
        #self.recorder_handle.record_trigger(self.trigger)
        if self.trigger.trigger == TestblockTrigger.PURGE:
            self._purge()
            new_state = TestblockState.PURGED
        elif self.trigger.trigger == TestblockTrigger.START:
            self._start()
            new_state = TestblockState.ACTIVE
        elif self.trigger.trigger == TestblockTrigger.STOP:
            rospy.logdebug("Stopping testblock is called from _paused_state")
            self._stop()
            new_state = TestblockState.SUCCEEDED
        else:
            message = "testblock '%s': invalid transition '%s' from state '%s'" % (self.trigger.name, str(self.trigger.trigger), self.get_state())
            rospy.logerr(message)
            new_state = TestblockState.ERROR
            self.exception = message
            raise ATFTestblockError(message)
        rospy.logdebug(" _paused_state trigger - _active_state : '%s'", self.trigger.trigger)
        self.trigger = None
        rospy.logdebug(" _paused_state after trigger = None : '%s'", self.trigger)
        rospy.logdebug("*** EXIT _paused_state ***")
        return new_state

    def _succeeded_state(self): # will never be called
        rospy.logdebug("*** ENTRY _succeeded_state ***")
        self._wait_while_transition_is_active()
        rospy.logdebug("*** EXIT _succeeded_state ***")

    def _error_state(self): # will never be called
        rospy.logdebug("*** ENTRY _error_state ***")
        self._wait_while_transition_is_active()
        rospy.logdebug("*** EXIT _error_state ***")

class ATFTestblockError(Exception):
    pass
