#!/usr/bin/python
import rospy
import smach
import smach_ros
import threading

from atf_core.sm_atf import SmAtfTestblock
from atf_core import ATFRecorder
import atf_core

#class Recorder():
#    def __init__(self):


#class Test(unittest.TestCase):
#    def setUp(self):
#        self.app = Application()

#    def tearDown(self):
#        pass

#    def test_Recording(self):
#        self.app.execute()

#if __name__ == '__main__':
#    rospy.init_node('test_name')
#    if "standalone" in sys.argv:
#        app = Application()
#        app.execute()
#    else:
#        rostest.rosrun('application', 'recording', Test, sysargs=None)


if __name__ == '__main__':
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded', 'error'])
    
    # get test config
    test_name = rospy.get_param("/atf/test_name")
    print "test_name:", test_name
    atf_configuration_parser = atf_core.ATFConfigurationParser()
    tests = atf_configuration_parser.get_tests()
    for test in tests:
        print "test.name:", test.name
        if test_name == test.name:
            break
    print "current test:", test.name
        
    tmp = {}
    for testblock in test.test_config.keys():
        tmp[testblock] = 'succeeded'
    outcome_map = {'succeeded':tmp}
    
    recorder_handle = ATFRecorder(test)

    # Open the container
    with sm_top:

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['succeeded','error'],
                                   default_outcome='error',
                                   outcome_map=outcome_map)

        sm_con.userdata = sm_top.userdata

        # Open the container
        with sm_con:
            # Add states to the container
            for testblock in test.test_config.keys():
                print "adding testblock:", testblock
                smach.Concurrence.add(testblock, SmAtfTestblock(testblock, recorder_handle))


        smach.StateMachine.add('CON', sm_con,
                                transitions={'succeeded':'succeeded',
                                            'error':'error'})

    sis = smach_ros.IntrospectionServer('/state_machine/machine', sm_top, 'SM_ATF')
    sis.start()

    # Execute SMACH plan
    print "before execute"
    outcome = sm_top.execute()
    print "after execute"

#    rospy.spin()

    sis.stop()
