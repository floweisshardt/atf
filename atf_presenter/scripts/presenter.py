#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import os
import sys
import optparse
import matplotlib.pyplot as plt


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
            for metric, data in metrics.iteritems():
                #print ("metric: ", metric)
                self.metric.add(metric)
                for values in data:
                    for name, result in values.iteritems():
                        #print ("name:", name)
                        if result != None:
                            #print ("result: ",result)
                            #print type(result)
                            if isinstance(result, dict):
                                for attribute, number in result.iteritems():
                                    #print ("attribute: ", attribute)
                                    if isinstance(number, list):
                                        print "numvals", numvals
                                        for item in number:
                                            #print ("value: ", item)
                                            if isinstance(item, str):
                                                continue
                                            else:
                                                numvals[metric].append(item)
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
                                    # elif isinstance(number, int):
                                    #     print ("integer: ",number)
                                    # elif isinstance(number, str) or isinstance(result, unicode):
                                    #     print ("string: ",number)
                            # elif isinstance(result, list):
                            #     for arrval in result:
                            #         print ("arrval:", arrval)
                            # elif isinstance(result, int):
                            #     print ("number: ", result)
                            # elif isinstance(result, str) or isinstance(result, unicode):
                            #     print ("string: ", result)
                            else:
                                rospy.logerr("Error, unknown result!")
        vals = [avg, numvals]
        #print "values", vals
        self.tests.update({str(num): vals})
        # print "-------------------------------------------------"
        # print "tests:\n", self.tests
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

    def show_results(self, single):
        if(single):
            print self.metric
            for metric in self.metric:
                (y_pos, means, devs) = self.calculate_data(metric)
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
        else:
            self.metric.remove('time')
            #print "metric: ",self.metric
            counter = 0
            fig, axarr = plt.subplots(len(self.metric), sharex=True)
            for metric in self.metric:
                (y_pos, means, devs) = self.calculate_data(metric)
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


    def calculate_data(self, metric):
        means = []
        devs = []
        for testname in sorted(self.tests, key=lambda ts : int(ts.split('_')[2].replace('r', ''))): # Magic!
            data = self.tests[testname]
            # print "-------------------------------------------"
            # print testname
            # print sorted(self.tests, key=lambda ts : int(ts.split('_')[2].replace('r', '')))
            # print "\n data: ", data
            # print "mean: ",data[0][metric]
            # print "values: ", data[1][metric]
            means.extend(data[0][metric])
            #for mean in data[1][metric]:
            #print "dev", numpy.std(data[1][metric]), "from mean", data[1][metric]
            devs.append(numpy.std(data[1][metric]))
            for test in self.testlist:
                #print "\n-------------------\ntest:", test, "\ntestname:", testname, "\n data: \n", data
                if testname in test:
                    #print "\n testname:", self.testlist, " \n \n test:", test
                    if (test[testname]['robot'] not in self.testnames):
                      self.testnames.append(test[testname]['robot'])
        # print("show")
        # print testnames
        # print self.metric
        # print plotdata
        y_pos = numpy.arange(len(self.testnames))
        # print "y pos", y_pos
        # print "height", means
        # print "deviation", devs
        return (y_pos, means, devs)

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('-s', '--single', dest='single', help='Print all plots in single windows', default=False, action="store_true")
    (options, args) = parser.parse_args()

    p = presenter()
    Path = "/home/fmw-hb/Desktop/hannes_test_slam/results_yaml/"#"/home/fmw-hb/Desktop/hannes_test_shortterm/results_yaml/"#"/tmp/hannes_test_new/results_yaml/"
    filelist = os.listdir(Path)
    p.import_testnames(Path.replace('yaml', 'json')+"test_list.json")

    for file in filelist:
        #print "file", file
        if "merged" in str(file):
            p.import_yaml(Path+file)
            filename = file.replace('.yaml', '')
            p.extract_yaml(filename.replace('merged_', ''))
    p.show_results(options.single)
