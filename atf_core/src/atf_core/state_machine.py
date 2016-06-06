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
        self.running = False
        self.finished = False

    def add_state(self, name, handler, end_state=False):
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name

    def get_current_state(self):
#        if self.handler == None:
#            return None
        return self.handlers.keys()[self.handlers.values().index(self.handler)]

    @threaded
    def run(self):
        try:
            self.handler = self.handlers[self.startState]
        except:
            raise SMError("Must call .set_start() before .run()")
        if not self.endStates:
            raise SMError("At least one state must be an end_state")

        self.running = True
        rospy.loginfo("-- SM for testblock '" + self.name + "' is running.")

        while not rospy.is_shutdown():
            new_state = self.handler()
            self.handler = self.handlers[new_state]
            rospy.loginfo("-- SM '" + self.name + "' in state " + str(self.get_current_state()))
            if new_state in self.endStates:
                break
        rospy.loginfo("-- SM '" + self.name + "' finished with state " + str(self.get_current_state()))
        self.finished = True
class SMError(Exception):
    pass
