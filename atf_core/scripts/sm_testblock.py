#!/usr/bin/python
import rospy
import smach
import smach_ros
import threading

from atf_core.sm_atf import SmAtfTestblock#Inactive, Active, Pause, Purge

if __name__ == '__main__':
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded', 'error'])
    
    sm_top.userdata.config = "initial config"
    testblocks = { #FIXME get this from the test config
        'TESTBLOCK1':{},
        'TESTBLOCK2':{},
        'TESTBLOCK3':{}}

    
    # Open the container
    with sm_top:

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['succeeded','error'],
                                   default_outcome='error',
                                   outcome_map={'succeeded':
                                       { 'TESTBLOCK1':'succeeded',
                                         'TESTBLOCK2':'succeeded'}},
                                   input_keys=['config'])

        sm_con.userdata.config = sm_top.userdata.config

        # Open the container
        with sm_con:
            # Add states to the container
            for testblock in testblocks:
                print "adding testblock:", testblock
                smach.Concurrence.add(testblock, SmAtfTestblock())


        smach.StateMachine.add('CON', sm_con,
                                transitions={'succeeded':'succeeded',
                                            'error':'error'})

    sis = smach_ros.IntrospectionServer('/state_machine/machine', sm_top, 'SM_ATF')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

#    rospy.spin()

    sis.stop()
