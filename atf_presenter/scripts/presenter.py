#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
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
        avg = copy.deepcopy(self.data)
        mini = copy.deepcopy(self.data)
        maxi = copy.deepcopy(self.data)
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
                                        print ("result: ",result)
                                        if attribute == "average":
                                            avg[metric].append(number)
                                            print "avg:", number
                                        if attribute == "min":
                                            mini[metric].append(number)
                                            print "min:", number
                                        if attribute == "max":
                                            maxi[metric].append(number)
                                            print "max:", number
                                        print "avg:", avg, "min:", mini, "max:", maxi
                                        print "\n \n \n -------------------------------------------------------------- \n \n \n"
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
        print "values", vals
        self.tests.update({"test"+str(num): vals})
        print "-------------------------------------------------"
        print ("tests:", self.tests)
        print "-------------------------------------------------"

    def show_results(self):
        print self.metric
        for metric in self.metric:
            plotdata = {}
            testnames = []
            means = []
            err = []
            for testname, data in self.tests.iteritems():
                # print "-------------------------------------------"
                # print testname
                print "data: ", data
                print "mean: ",data[0][metric]
                print "max: ", data[1][metric]
                # #plotdata.update({testname : data[metric]})
                means.extend(data[0][metric])
                err.extend(numpy.array(data[1][metric])-numpy.array(data[0][metric]))
                print "err:", err
                for i in data[0][metric]:
                    testnames.append(testname)


            # print("show")
            print testnames
            # print self.metric
            # print plotdata

            fig, ax = pyplot.subplots()
            y_pos = numpy.arange(len(testnames))
            print "x", y_pos
            print "y", means
            #pyplot.bar(y_pos, means, align='center', alpha=0.5, yerr=err)
            # example data
            x = numpy.arange(0.1, 2, 0.5)
            y = numpy.exp(-x)
            print "x", x
            print "y", y

            # example variable error bar values
            yerr = 0.1 + 0.2 * numpy.sqrt(x)
            xerr = 0.1 + yerr
            #pyplot.errorbar(y_pos, means, yerr=[yerr, 2 * yerr], fmt='--o')
            #pyplot.bar(y_pos, means, yerr=[yerr, 4 * yerr], alpha=0.5)
            print "ypos:", y_pos[0], "means", means[0]
            rects1 = ax.bar(y_pos, means, yerr=[yerr, 4 * yerr], alpha=0.5)
            rects2 = ax.bar(y_pos, means, yerr=[yerr, 0.5 * yerr], alpha=0.5)

            # pyplot.xticks(y_pos, testnames)
            # pyplot.ylabel('Usage')
            # pyplot.title(metric)

            ax.set_ylabel('Scores')
            ax.set_title('Scores by group and gender')
            ax.set_xticks(y_pos + 0.5 / 2)
            #ax.set_xticklabels(('G1', 'G2', 'G3', 'G4', 'G5'))

            #ax.legend((rects1[0], rects2[0]), ('Men', 'Women'))

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
