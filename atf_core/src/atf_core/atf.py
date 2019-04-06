#!/usr/bin/env python
import rospy
import atf_core

from atf_msgs.msg import TestblockStatus

class ATF:
    def __init__(self):
        #package_name = rospy.get_param("/atf/package_name")
        #print "package_name:", package_name

        self.initialized = False
        self.finished = False

        test_name = rospy.get_param("/atf/test_name")
        print "test_name:", test_name

        atf_configuration_parser = atf_core.ATFConfigurationParser()

        tests_without_recorders = atf_configuration_parser.get_tests()
        #print "tests_without_recorders:", tests_without_recorders
        for test in tests_without_recorders:
            print "test.name:", test.name
            if test_name == test.name:
                break
        print "current test:", test.name
        self.test = test

        print "self.test.generation_config:", self.test.generation_config
        print "self.test.robot_config:", self.test.robot_config

        recorder_handle = atf_core.ATFRecorder(test)
        self.test.create_recorder(recorder_handle)

        #self.config = atf_configuration_parser.get_config()
        #print "atf: config=", self.config
        #self.testblock_list = atf_configuration_parser.create_testblock_list(self.config)
        #print "atf: testblock_list=", self.testblock_list


        #self.testblocks = atf_configuration_parser.create_testblocks(self.config, self.recorder_handle)
        self.init()

    def init(self):
        if self.finished:
            raise ATFError("Calling ATF init while ATF is already finished.")
        if self.initialized:
            raise ATFError("Calling ATF init while ATF is already initialized.")
        self.initialized = True
        for testblock in self.test.testblocks:
            testblock._run()

    def start(self, current_testblock):
        if self.finished:
            raise ATFError("Calling ATF start for testblock '%s' while ATF is already finished." % current_testblock)
        if not self.initialized:
            raise ATFError("Calling ATF start for testblock '%s' before ATF has been initialized." % current_testblock)
        self.initialized = True
        if current_testblock not in self.test.test_config:
            raise ATFError("Testblock '%s' not in test_config." % current_testblock)

        for testblock in self.test.testblocks:
            if testblock.name == current_testblock:
                testblock.start()

    def stop(self, current_testblock):
        for testblock in self.test.testblocks:
            if testblock.name == current_testblock:
                testblock.stop()

    def shutdown(self):
        rospy.loginfo("Shutdown ATF.")
        if not self.initialized:
            raise ATFError("Calling ATF finish before ATF has been initialized.")
        if self.finished:
            raise ATFError("Calling ATF finish while ATF is already finished.")
        self.finished = True

        # stop testblocks if not already stopped
        for testblock in self.test.testblocks:
            if testblock.get_state() not in testblock.m.endStates and testblock.trigger == None:
                rospy.logwarn("Stopping testblock '%s' automatically because ATF stop is trigged and testblock is not in an end state.", testblock.name)
                testblock.stop()

        # wait for all testblocks to finish
        for testblock in self.test.testblocks:
            r = rospy.Rate(10)
            while not testblock._finished():
                r.sleep()
                continue

        # check end state for each testblock
        error = False
        message = ""
        for testblock in self.test.testblocks:
            state = testblock.get_state()
            if not state == TestblockStatus.SUCCEEDED:
                error = True
                message += "Testblock '%s' did not succeed (finished with state '%d');\n " % (testblock.name, state)
        if error:
            raise ATFError(message)

class ATFError(Exception):
    pass
