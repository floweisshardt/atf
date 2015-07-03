#!/usr/bin/env python

import rospy

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.handler = None
        self.endStates = []

    def add_state(self, name, handler, end_state=False):
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name

    def get_current_state(self):
        return self.handlers.keys()[self.handlers.values().index(self.handler)]

    def get_current_state_name(self, state):
        return state.__name__

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
                self.handler = self.handlers[newState]
                break 
            else:
                self.handler = self.handlers[newState]
            print "...sm in state", self.get_current_state(), "..."
        print "...sm finished with state", self.get_current_state()
                


