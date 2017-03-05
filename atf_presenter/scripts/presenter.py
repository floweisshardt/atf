#!/usr/bin/env python
import numpy
import rospy
import yaml
import matplotlib.pyplot as pyplot

class presenter:

    def __init__(self):
        self.filepath = "/tmp/atf_test/results_yaml/"
        self.yaml_file = {}
        self.testblock = []
        self.metric = set()
        self.data = {}
        self.tests = {}



    def import_yaml(self, file):
        with open(self.filepath+file, 'r') as stream:
            try:
                #print(yaml.load(stream))
                self.yaml_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        for testblock, metrics in self.yaml_file.iteritems():
            for metric, values in metrics.iteritems():
                self.data.update({metric : []})

    def extract_yaml(self, num):
        avg = self.data.copy()
        mini = self.data.copy()
        maxi = self.data.copy()
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
                                        for dictval in number:
                                            print ("value: ", dictval)
                                    elif isinstance(number, float):
                                        #print ("float: ",number)
                                        if attribute == "average":
                                            avg[metric].append(number)
                                        if attribute == "min":
                                            mini[metric].append(number)
                                        if attribute == "max":
                                            maxi[metric].append(number)
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
        vals = [avg, maxi, mini]
        self.tests.update({"test"+str(num): vals})
        print ("tests:", self.tests)
    def show_results(self):
        print self.metric
        for metric in self.metric:
            plotdata = {}
            testnames = []
            for testname, data in self.tests.iteritems():
                print "-------------------------------------------"
                print testname
                print data
                #plotdata.update({testname : data[metric]})
                means = data[0][metric]
                for i in (means):
                    testnames.append(testname)


            print("show")
            print testnames
            print self.metric
            print plotdata
            N = 5
            ind = numpy.arange(len(data[0][metric]))  # the x locations for the groups
            width = 0.35  # the width of the bars

            #fig, ax = pyplot.subplots()
            rects1 = ax.bar(ind, means, width, color='r')#, yerr=men_std)

            # women_means = [25, 32, 34, 20, 25]
            # women_std = [3, 5, 2, 3, 3]
            # ind = numpy.arange(N)
            # rects2 = ax.bar(ind + width, women_means, width, color='y', yerr=women_std)

            # add some text for labels, title and axes ticks
            ax.set_ylabel('Scores')
            ax.set_title(metric)
            ax.set_xticks(ind + width / 2)
            ax.set_xticklabels(testnames)

            ax.legend((rects1[0], rects2[0]), ('Men', 'Women'))

            def autolabel(rects):
                """
                Attach a text label above each bar displaying its height
                """
                for rect in rects:
                    height = rect.get_height()
                    ax.text(rect.get_x() + rect.get_width() / 2., 1.05 * height,
                            '%d' % int(height),
                            ha='center', va='bottom')

            autolabel(rects1)
            autolabel(rects2)

            pyplot.show()

if __name__ == '__main__':
    p = presenter()
    p.import_yaml("merged_ts0_c0_r0_e0.yaml")
    p.extract_yaml(1)
    p.import_yaml("merged_ts0_c0_r1_e0.yaml")
    p.extract_yaml(2)
    p.show_results()
