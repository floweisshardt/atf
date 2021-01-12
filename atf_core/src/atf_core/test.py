#!/usr/bin/env python

from atf_msgs.msg import TestResult
from atf_metrics.error import ATFAnalyserError

class Test:
    def __init__(self):
        self.package_name = None
        self.name = None
        self.generation_config = None
        self.testsuite = None
        self.test_config = None
        self.test_config_name = None
        self.robot_config = None
        self.robot_name = None
        self.env_config = None
        self.env_name = None
        self.testblockset_config = None
        self.testblockset_name = None

        # testblocks with metrics
        self.testblocks = []
    
    def get_result(self):
        test_result = TestResult()
        test_result.name = self.name
        test_result.robot = self.robot_name
        test_result.env = self.env_name
        test_result.test_config = self.test_config_name
        test_result.testblockset = self.testblockset_name
        test_result.result = None
        for testblock in self.testblocks:
            # get result
            testblock_result = testblock.get_result()

            # append result
            test_result.results.append(testblock_result)

            # aggregate result
            if testblock_result.result != None and not testblock_result.result:
                test_result.result = False
                test_result.error_message += "\n   - testblock '%s': %s"%(testblock_result.name, testblock_result.error_message)
                #print test_result.error_message
            if test_result.result == None and testblock_result.result:
                test_result.result = True
            
        if test_result.result == None:
            raise ATFAnalyserError("Analysing failed, test result is None for test '%s'."%test_result.name)

        if len(test_result.results) == 0:
            raise ATFAnalyserError("Analysing failed, no test result available for test '%s'."%test_result.name)

        #print "\ntest_result:\n", test_result
        return test_result
