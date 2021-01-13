#!/usr/bin/env python
import rospy
import rostest
import smach
import smach_ros
import sys
import unittest

import atf_core
from atf_core.configuration_parser import ATFConfigurationParser
from atf_core.recorder import ATFRecorder
from atf_core.sm_atf import SmAtfTestblock

class Recorder():
    def __init__(self):
        #rospy.init_node('state_machine')

        # Create a SMACH state machine
        self.sm_top = smach.StateMachine(outcomes=['succeeded', 'error'])
        
        # get test config
        package_name = rospy.get_param("/atf/package_name")
        print("package_name:", package_name)
        test_generation_config_file = rospy.get_param("/atf/test_generation_config_file", "atf/test_generation_config.yaml")
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
            
        outcome_map_succeeded = {}
        outcome_map_error = {}
        for testblock in list(test.testblockset_config.keys()):
            outcome_map_succeeded[testblock] = 'succeeded'
            outcome_map_error[testblock] = 'error'
        outcome_map = {'succeeded':outcome_map_succeeded,
                        'error':outcome_map_error}
        
        recorder_handle = ATFRecorder(test)

        # Open the container
        with self.sm_top:

            # Create the sub SMACH state machine
            sm_con = smach.Concurrence(outcomes=['succeeded','error'],
                                       default_outcome='error',
                                       outcome_map=outcome_map)

            sm_con.userdata = self.sm_top.userdata

            # Open the container
            with sm_con:
                # Add states to the container
                for testblock in list(test.testblockset_config.keys()):
                    #print "adding testblock:", testblock
                    smach.Concurrence.add(testblock, SmAtfTestblock(testblock, recorder_handle))
            
            # TODO preempt all other concurrent States as soon as one state returns 'error'

            smach.StateMachine.add('CON', sm_con,
                                    transitions={'succeeded':'succeeded',
                                                'error':'error'})

        self.sis = smach_ros.IntrospectionServer('/state_machine/machine', self.sm_top, 'SM_ATF')
        self.sis.start()
        
    def execute(self):
        outcome = self.sm_top.execute()

class Test(unittest.TestCase):

    def setUp(self):
        self.rec = Recorder()

    def tearDown(self):
        self.rec.sis.stop()
        pass

    def test_Recording(self):
        self.assertEqual(self.rec.sm_top.execute(), 'succeeded')

if __name__ == '__main__':
    rospy.init_node('test_name')
    if "execute_as_test" in sys.argv:
        rostest.rosrun('application', 'recording', Test)
    else:
        rec = Recorder()
        # Execute SMACH plan
        rec.sm_top.execute()
        rec.sis.stop()
