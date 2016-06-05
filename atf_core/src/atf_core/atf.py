#!/usr/bin/env python
import rospy
import atf_core

from atf_msgs.msg import TestblockState

class ATF:
    def __init__(self, mode = "recording"):
        self.testblocks = {}
        self.started = False
        self.stopped = False
        atf_configuration_parser = atf_core.ATFConfigurationParser()
        self.config = atf_configuration_parser.config
        #print "atf: config=", self.config

        # check for mode (recording or analysing)
        #print "atf: mode=", mode
        if mode == "recording":
            self.recorder_handle = atf_core.ATFRecorder(self.config)
        elif mode == "analysing":
            pass
        else:
            raise TestblockError("unknown mode '%s', supported modes are 'recording' and 'analysing'." % mode)
        

    def add_testblock(self, name):
        #print "atf: test_config.keys=", self.config["test_config"][self.config["test_config_name"]].keys()
        if name in self.config["test_config"][self.config["test_config_name"]].keys():
            #print "add", name
            metrics = self.config["test_config"][self.config["test_config_name"]][name]
            #print "metrics=", metrics
            self.testblocks[name] = atf_core.Testblock(name, [], self.recorder_handle)
        else:
            raise ATFError("Testblock '%s' not in test config." % name)

    def start(self):
        if self.stopped:
            raise ATFError("Calling ATF start while ATF is already stopped.")
        if self.started:
            raise ATFError("Calling ATF start while ATF is already started.")
        self.started = True
        for testblock in self.testblocks.values():
            testblock._run()
            
    def stop(self):
        if not self.started:
            raise ATFError("Calling ATF stop before ATF has been started.")
        if self.stopped:
            raise ATFError("Calling ATF stop while ATF is already stopped.")
        self.stopped = True

        # stop testblocks if not already stopped
        for testblock_name, testblock in self.testblocks.items():
            if testblock.get_state() not in testblock.m.endStates and testblock.trigger == None:
                rospy.logwarn("Stopping testblock '%s' automatically because ATF stop is trigged and testblock is not in an end state.", testblock_name)
                testblock.stop()
        
        # wait for all testblocks to finish
        for testblock_name, testblock in self.testblocks.items():
            while not testblock._finished():
                continue

        # check result for each testblock
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
