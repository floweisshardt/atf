#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import matplotlib.pyplot as plt

import os

class presenter:

    def __init__(self):
        self.filepath = "/tmp/atf_test/results_yaml/"
        self.yaml_file = {}
        self.testblock = []
        self.metric = set()
        self.data = {}
        self.tests = {}
        self.testnames = []
        self.testlist = {}



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
                                        print "numvals", numvals
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
        print "values", vals
        self.tests.update({str(num): vals})
        # print "-------------------------------------------------"
        # print ("tests:", self.tests)
        # print "-------------------------------------------------"

    def import_testnames(self, file):
        with open(file, 'r') as stream:
            # testlist = yaml.load(stream)
            # #print testlist
            # for test in testlist:
            #     for robot in test.iteritems():
            #         print "robot:", robot[1]['robot']
            #         self.testnames.append(robot[1]['robot'])
            self.testlist = yaml.load(stream)

    def show_results(self, all):
        if(all == "all"):
            self.metric.remove('time')
            #print "metric: ",self.metric
            counter = 0
            fig, axarr = plt.subplots(len(self.metric), sharex=True)
            testnames = []
            for metric in self.metric:
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
                    for test in self.testlist:
                        if testname in test:
                            #print "\n testname:", self.testlist, " \n \n test:", test
                            if (test[testname]['robot'] not in self.testnames):
                              self.testnames.append(test[testname]['robot'])
                #print "------------------------\n testnames:", testnames, "\n", self.testnames


                # print("show")
                # print testnames
                # print self.metric
                # print plotdata
                y_pos = numpy.arange(len(self.testnames))
                # print "x", y_pos
                # print "y", means
                # print "yerr", devs

                # plt.bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                # plt.xticks(y_pos+0.8/2, testnames, rotation='vertical')
                # plt.title(metric)
                # plt.tight_layout()

                axarr[counter].bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                rects = axarr[counter].bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                axarr[counter].set_title(metric)
                axarr[counter].set_xticks(y_pos+0.8/2)
                axarr[counter].set_xticklabels(self.testnames, rotation='vertical')
                for rect in rects:
                    height = rect.get_height()
                    axarr[counter].text(rect.get_x() + rect.get_width()/2., 1.05*height,
                            '%.1f' % round(height, 1),
                            ha='center', va='bottom')
                counter += 1
            plt.tight_layout()
            plt.show()
        else:
            print self.metric
            for metric in self.metric:
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
                    for test in self.testlist:
                        if testname in test:
                            #print "\n testname:", self.testlist, " \n \n test:", test
                            if (test[testname]['robot'] not in self.testnames):
                              self.testnames.append(test[testname]['robot'])


                # print("show")
                # print testnames
                # print self.metric
                # print plotdata
                y_pos = numpy.arange(len(self.testnames))
                # print "x", y_pos
                # print "y", means
                # print "yerr", devs

                plt.bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                rects = plt.bar(y_pos, means, yerr=devs, alpha=0.5, color='red')
                for rect in rects:
                    height = rect.get_height()
                    plt.text(rect.get_x() + rect.get_width()/2., 1.05*height,
                            '%.1f' % round(height, 1),
                            ha='center', va='bottom')
                plt.xticks(y_pos+0.8/2, self.testnames, rotation='vertical')
                plt.title(metric)
                plt.tight_layout()
                plt.tight_layout()
                plt.show()


if __name__ == '__main__':
    p = presenter()

    Path = "/tmp/hannes_test_new/results_yaml/"
    filelist = os.listdir(Path)
    p.import_testnames(Path.replace('yaml', 'json')+"test_list.json")

    for file in filelist:
        #print "file", file
        #if "merged" in str(file):
        p.import_yaml(Path+file)
        filename = file.replace('.yaml', '')
        p.extract_yaml(filename.replace('merged_', ''))
    p.show_results("all")
