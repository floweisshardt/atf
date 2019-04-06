#!/usr/bin/env python
import rospy
from atf_core.atf import ATFError
import atf_core

from atf_msgs.msg import TestblockTrigger

######################
### ATF controller ###
######################
class ATFController():
    def __init__(self):
        test_name = rospy.get_param("/atf/test_name")
        print "test_name:", test_name
        atf_configuration_parser = atf_core.ATFConfigurationParser()
        tests = atf_configuration_parser.get_tests()
        for test in tests:
            print "test.name:", test.name
            if test_name == test.name:
                break
        print "current test:", test.name
        self.test = test

        self.publisher = {}
        #for testblock in self.test.test_config.keys():
        #    print "testblock", testblock
        self.publisher = rospy.Publisher("atf/trigger", TestblockTrigger, queue_size=10)

        rospy.sleep(1) #wait for all publishers to be ready
    
    def start(self, testblock):
        if testblock not in self.test.test_config.keys():
            raise ATFError("testblock %s not in list of testblocks"%testblock)
        print "starting testblock", testblock
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.START
        self.publisher.publish(trigger)

    def stop(self, testblock):
        if testblock not in self.test.test_config.keys():
            raise ATFError("testblock %s not in list of testblocks"%testblock)
        print "stopping testblock", testblock
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.STOP
        self.publisher.publish(trigger)
