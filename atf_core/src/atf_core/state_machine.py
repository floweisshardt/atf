#!/usr/bin/env python
import rospy

from threading import Thread


def threaded(fn):
    def wrapper(*args, **kwargs):
        Thread(target=fn, args=args, kwargs=kwargs).start()

    return wrapper


class StateMachine:
    def __init__(self, name):
        self.handlers = {}
        self.startState = None
        self.handler = None
        self.endStates = []
        self.name = name

    def add_state(self, name, handler, end_state=False):
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name

    def get_current_state(self):
        return self.handlers.keys()[self.handlers.values().index(self.handler)]

    @staticmethod
    def get_current_state_name(state):
        return state.__name__

    @threaded
    def run(self):
        try:
            self.handler = self.handlers[self.startState]
        except:
            raise "InitializationError", "Must call .set_start() before .run()"
        if not self.endStates:
            raise "InitializationError", "At least one state must be an end_state"

        rospy.loginfo("SM for testblock '" + self.name + "' is running")

        while not rospy.is_shutdown():
            new_state = self.handler()
            if new_state in self.endStates:
                self.handler = self.handlers[new_state]
                break
            else:
                self.handler = self.handlers[new_state]
            rospy.loginfo("SM '" + self.name + "' in state " + str(self.get_current_state()))
        rospy.loginfo("SM '" + self.name + "' finished with state " + str(self.get_current_state()))
