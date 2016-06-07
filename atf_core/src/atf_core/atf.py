#!/usr/bin/env python
import rospy
import atf_core

from atf_msgs.msg import TestblockState

class ATF:
    def __init__(self):
        self.initialized = False
        self.finished = False
        atf_configuration_parser = atf_core.ATFConfigurationParser()
        self.config = atf_configuration_parser.get_config()
        #print "atf: config=", self.config
        self.testblock_list = atf_configuration_parser.create_testblock_list(self.config)
        #print "atf: testblock_list=", self.testblock_list

        self.recorder_handle = atf_core.ATFRecorder(self.config, self.testblock_list)
        self.testblocks = atf_configuration_parser.create_testblocks(self.config, self.recorder_handle)
        self.init()

    def init(self):
        if self.finished:
            raise ATFError("Calling ATF init while ATF is already finished.")
        if self.initialized:
            raise ATFError("Calling ATF init while ATF is already initialized.")
        self.initialized = True
        for testblock in self.testblocks.values():
            testblock._run()

    def start(self, testblock):
        if self.finished:
            raise ATFError("Calling ATF start for testblock '%s' while ATF is already finished." % testblock)
        if not self.initialized:
            raise ATFError("Calling ATF start for testblock '%s' before ATF has been initialized." % testblock)
        self.initialized = True
        if testblock not in self.config["test_config"]:
            raise ATFError("Testblock '%s' not in test_config." % testblock)
        self.testblocks[testblock].start()

    def stop(self, testblock):
        self.testblocks[testblock].stop()

    def shutdown(self):
        rospy.loginfo("Shutdown ATF.")
        if not self.initialized:
            raise ATFError("Calling ATF finish before ATF has been initialized.")
        if self.finished:
            raise ATFError("Calling ATF finish while ATF is already finished.")
        self.finished = True

        # stop testblocks if not already stopped
        for testblock_name, testblock in self.testblocks.items():
            if testblock.get_state() not in testblock.m.endStates and testblock.trigger == None:
                rospy.logwarn("Stopping testblock '%s' automatically because ATF stop is trigged and testblock is not in an end state.", testblock_name)
                testblock.stop()

        # wait for all testblocks to finish
        for testblock_name, testblock in self.testblocks.items():
            r = rospy.Rate(10)
            while not testblock._finished():
                r.sleep()
                continue

        # check end state for each testblock
        error = False
        message = ""
        for testblock_name, testblock in self.testblocks.items():
            state = testblock.get_state()
            if not state == TestblockState.SUCCEEDED:
                error = True
                message += "Testblock '%s' did not succeed (finished with state '%d');\n " % (testblock_name, state)
        if error:
            raise ATFError(message)

class ATFError(Exception):
    pass
