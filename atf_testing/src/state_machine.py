#!/usr/bin/env python

import rospy
from atf_msgs.msg import Status, Trigger

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.handler = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name

    def get_current_state(self):
        return self.handler.__name__

    def run(self):
        try:
            self.handler = self.handlers[self.startState]
        except:
            raise InitializationError("must call .set_start() before .run()")
        if not self.endStates:
            raise InitializationError("at least one state must be an end_state")

        print "sm running..."    
        while not rospy.is_shutdown():
            newState = self.handler()
            if newState in self.endStates:
                break 
            else:
                self.handler = self.handlers[newState]
        print "...sm finished"
                


