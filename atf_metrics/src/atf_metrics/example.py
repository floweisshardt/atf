#!/usr/bin/env python


class ExampleParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    @staticmethod
    def parse_parameter(params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """

        return Example(params)


class Example:
    def __init__(self, params):
        self.finished = False

    def start(self):
        pass

    def stop(self):
        self.finished = True

    @staticmethod
    def pause():
        pass

    @staticmethod
    def purge():
        pass

    def get_result(self):
        if self.finished:
            return "example", "results"
        else:
            return False
