#!/usr/bin/env python
import rospy
import copy

from atf_core.error import ATFError
from atf_core.configuration_parser import ATFConfigurationParser
from atf_metrics.error import ATFConfigurationError
from atf_msgs.msg import MetricResult, TestblockTrigger, DataStamped
from smach_msgs.msg import SmachContainerStatus


###########
### ATF ###
###########
class ATF():
    def __init__(self):
        # get test config
        package_name = rospy.get_param("/atf/package_name")
        print("package_name:", package_name)
        test_generation_config_file = rospy.get_param("/atf/test_generation_config_file")
        print("test_generation_config_file:", test_generation_config_file)
        test_name = rospy.get_param("/atf/test_name")
        print("test_name:", test_name)

        atf_configuration_parser = ATFConfigurationParser(package_name, test_generation_config_file)
        tests = atf_configuration_parser.get_tests()
        for test in tests:
            #print "test.name:", test.name
            if test_name == test.name:
                break
        #print "current test:", test.name
        self.test = test

        self.publisher_trigger = rospy.Publisher("atf/trigger", TestblockTrigger, queue_size=10)
        rospy.Subscriber("/state_machine/machine/smach/container_status", SmachContainerStatus, self._sm_status_cb)
        self.sm_container_status = None

        # make sure to wait with the application for the statemachine in sm_test.py to be initialized
        rospy.loginfo("waiting for smach container in test_sm to be ready...")
        rospy.wait_for_message("/state_machine/machine/smach/container_status", rospy.AnyMsg)
        rospy.sleep(1) # wait for sm_test to initialize all subscribers (rospy bug?)
        rospy.loginfo("...smach container in sm_test is ready.")
    
    def start(self, testblock):
        if testblock not in list(self.test.testblockset_config.keys()):
            error_msg = "testblock \'%s\' not in list of testblocks"%testblock
            self._send_error(error_msg)
            raise ATFError(error_msg)
        rospy.loginfo("starting testblock \'%s\'"%testblock)
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.START
        self.publisher_trigger.publish(trigger)

    def stop(self, testblock, metric_result = None):
        if testblock not in list(self.test.testblockset_config.keys()):
            error_msg = "testblock \'%s\' not in list of testblocks"%testblock
            self._send_error(error_msg)
            raise ATFError(error_msg)

        if metric_result != None:

            #TODO  check if metric_result is of type atf_msgs/MetricResult

            metric_result = copy.deepcopy(metric_result) # deepcopy is needed to be able to overwrite metric_result.name
            metric_result.name = "user_result"
            
            rospy.loginfo("setting user result for testblock \'%s\'"%testblock)
            if not isinstance(metric_result, MetricResult):
                error_msg = "metric_result of testblock %s for " \
                            "metric %s is not a MetricResult. data=%s, type=%s" % (
                            testblock, metric_result.name, str(metric_result), type(metric_result))
                self._send_error(error_msg)
                raise ATFError(error_msg)
            if not isinstance(metric_result.data, DataStamped):
                error_msg = "metric_result.data of testblock %s for " \
                            "metric %s is not a DataStamped. data=%s, type=%s" % (
                            testblock, metric_result.name, str(metric_result.data), type(metric_result.data))
                self._send_error(error_msg)
                raise ATFError(error_msg)
            if not isinstance(metric_result.data.data, float) and not isinstance(metric_result.data.data, int):
                error_msg = "metric_result.data.data of testblock %s for " \
                            "metric %s is not a float or int. data=%s, type=%s" % (
                    testblock, metric_result.name, str(metric_result.data.data), type(metric_result.data.data))
                self._send_error(error_msg)
                raise ATFError(error_msg)
            if type(metric_result.details) is not list:
                error_msg = "metric_result.details of testblock %s for " \
                            "metric %s is not a list. detail=%s" % (
                    testblock, metric_result.name, str(metric_result.details))
                self._send_error(error_msg)
                raise ATFError(error_msg)

        else:
            rospy.loginfo("no user result set for testblock \'%s\'"%testblock)
            metric_result = MetricResult()

        rospy.loginfo("stopping testblock \'%s\'"%testblock)
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = testblock
        trigger.trigger = TestblockTrigger.STOP
        trigger.user_result = metric_result
        self.publisher_trigger.publish(trigger)

    def shutdown(self):
        rospy.loginfo("shutting down atf application")

        # call stop for all testblocks
        for testblock in self.test.testblocks:
            rospy.sleep(1) # TODO remove sleep and wait until all states have finished their transitions
            if testblock.name in self.sm_container_status.active_states:
                rospy.logwarn("shutdown called but testblock %s is still active: calling stop automatically"%testblock.name)
                self.stop(testblock.name)

        # check if any testblock is still running
        rospy.loginfo("waiting for all states to be in a terminal state")
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.sm_container_status != None and len(self.sm_container_status.active_states) == 0:
                rospy.logdebug("all testblocks finished in SM_ATF/CON")
                break
            rospy.logdebug("still waiting for active states in path 'SM_ATF/CON' to be prepempted. active_states: %s", str(self.sm_container_status.active_states))
            r.sleep()
            continue
        rospy.loginfo("atf application is shutdown.")

    def _send_error(self, error_msg):
        trigger = TestblockTrigger()
        trigger.stamp = rospy.Time.now()
        trigger.name = error_msg
        trigger.trigger = TestblockTrigger.ERROR
        self.publisher_trigger.publish(trigger)

    def _sm_status_cb(self, msg):
        if msg.path == "SM_ATF/CON":
            self.sm_container_status = msg
