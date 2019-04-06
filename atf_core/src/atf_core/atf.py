#!/usr/bin/env python
import rospy
import atf_core

from atf_msgs.msg import TestblockTrigger

###########
### ATF ###
###########
class ATF():
    def __init__(self):
        # get test config
        package_name = rospy.get_param("/atf/package_name")
        print "package_name:", package_name
        test_name = rospy.get_param("/atf/test_name")
        print "test_name:", test_name

        atf_configuration_parser = atf_core.ATFConfigurationParser(package_name)
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
            error_msg = "testblock %s not in list of testblocks"%testblock
            self.error(error_msg)
            raise atf_core.ATFError(error_msg)
        print "starting testblock", testblock
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.START
        self.publisher.publish(trigger)

    def stop(self, testblock):
        if testblock not in self.test.test_config.keys():
            error_msg = "testblock %s not in list of testblocks"%testblock
            self.error(error_msg)
            raise atf_core.ATFError(error_msg)
        print "stopping testblock", testblock
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.STOP
        self.publisher.publish(trigger)
        
    def error(self, error_msg):
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = error_msg
        trigger.trigger = TestblockTrigger.ERROR
        self.publisher.publish(trigger)
