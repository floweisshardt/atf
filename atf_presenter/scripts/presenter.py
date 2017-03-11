#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import matplotlib.pyplot as pyplot

import os

class presenter:

    def __init__(self):
        self.filepath = "/tmp/atf_test/results_yaml/"
        self.yaml_file = {}
        self.testblock = []
        self.metric = set()
        self.data = {}
        self.tests = {}



    def import_yaml(self, file):
        with open(file, 'r') as stream:
            print "import file", file
            try:
                #print(yaml.load(stream))
                self.yaml_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        for testblock, metrics in self.yaml_file.iteritems():
            for metric, values in metrics.iteritems():
                self.data.update({metric : []})

    def extract_yaml(self, num):
        avg = copy.deepcopy(self.data)
        mini = copy.deepcopy(self.data)
        maxi = copy.deepcopy(self.data)
        numvals = copy.deepcopy(self.data)
        for testblock, metrics in self.yaml_file.iteritems():
            #print ("testblock: ", testblock)
            for metric, values in metrics.iteritems():
                #print ("metric: ", metric)
                self.metric.add(metric)
                for data in values:
                    for name, result in data.iteritems():
                        #print ("name:", name)
                        if result != None:
                            #print ("result: ",result)
                            #print type(result)
                            if isinstance(result, dict):
                                for attribute, number in result.iteritems():
                                    #print ("attribute: ", attribute)
                                    if isinstance(number, list):
                                        numvals[metric].append(number)
                                        #print "numvals", numvals
                                        #for dictval in number:
                                            #print ("value: ", dictval)
                                    elif isinstance(number, float):
                                        #print ("result: ",result)
                                        if attribute == "average":
                                            avg[metric].append(number)
                                            ##print "avg:", number
                                        if attribute == "min":
                                            mini[metric].append(number)
                                            #print "min:", number
                                        if attribute == "max":
                                            maxi[metric].append(number)
                                            #print "max:", number
                                        #print "avg:", avg, "min:", mini, "max:", maxi
                                        #print "\n \n -------------------------------------------------------------- \n \n"
                                    elif isinstance(number, int):
                                        print ("integer: ",number)
                                    elif isinstance(number, str) or isinstance(result, unicode):
                                        print ("string: ",number)
                            elif isinstance(result, list):
                                for arrval in result:
                                    print ("arrval:", arrval)
                            elif isinstance(result, int):
                                print ("number: ", result)
                            elif isinstance(result, str) or isinstance(result, unicode):
                                print ("string: ", result)
                            else:
                                rospy.logerr("Error, unknown result!")
        vals = [avg, numvals]
        #print "values", vals
        self.tests.update({str(num): vals})
        # print "-------------------------------------------------"
        # print ("tests:", self.tests)
        # print "-------------------------------------------------"

    def show_results(self):
        print self.metric
        for metric in self.metric:
            testnames = []
            means = []
            devs = []
            for testname, data in self.tests.iteritems():
                # print "-------------------------------------------"
                # print testname
                # print "\n data: ", data
                # print "\n mean: ",data[0][metric]
                # print "\n values: ", data[1][metric]
                means.extend(data[0][metric])
                for mean in data[1][metric]:
                    #print "mean", mean
                    devs.append(numpy.std(mean))
                for i in data[0][metric]:
                    testnames.append(testname)

            # print("show")
            #print testnames
            # print self.metric
            # print plotdata

            y_pos = numpy.arange(len(testnames))
            print "x", y_pos
            print "y", means
            print "yerr", devs

            pyplot.bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
            pyplot.xticks(y_pos+0.8/2, testnames, rotation='vertical')
            pyplot.title(metric)
            pyplot.tight_layout()
            pyplot.show()

if __name__ == '__main__':
    p = presenter()

    Path = "/tmp/atf_test/results_yaml/"
    filelist = os.listdir(Path)
    for file in filelist:
        print "file", file
        if "merged" in str(file):
            print "merged", file
            p.import_yaml(Path+file)
            filename = file.replace('.yaml', '')
            p.extract_yaml(filename.replace('merged_', ''))
    p.show_results()
