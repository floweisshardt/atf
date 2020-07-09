#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import matplotlib.style
import matplotlib as mpl
mpl.style.use('classic')

from atf_msgs.msg import AtfResult

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


import yaml

import rosbag

import pprint
import re

import numpy as np

from matplotlib import cm

from collections import namedtuple

import argparse

import sys

import time

MetricAggregate = namedtuple('MetricAggregate', 'metric_name testblock_name num min max mean median std data_points')

class DemoPlotter(object):

    def __init__(self):
        self.atf_result = None


    def load_atf_result(self, filename):
        
        bag = rosbag.Bag(filename)
        print "number of files in bag:", bag.get_message_count()
        assert bag.get_message_count() == 1 # check if only one message is in the bag file
            
        for topic, msg, t in bag.read_messages():
            #print type(msg), type(AtfResult()), isinstance(msg, AtfResult) # check if message is an AtfResult message
            #assert isinstance(msg, AtfResult) # check if message is an AtfResult message
            self.atf_result = msg
            bag.close()
            break

    def plot_fmw(self):
        #print "self.atf_result=", self.atf_result.groundtruth_error_message
        #print "type(atf_result)=", type(self.atf_result)


        tbm = {}
        tmb = {}
        for test in self.atf_result.results:
            #print test.name
            tbm[test.name] = {}
            tmb[test.name] = {}
            for testblock in test.results:
                #print "  -", testblock.name
                tbm[test.name][testblock.name] = {}
                for metric in testblock.results:
                    #print "    -", metric.name
                    if metric.name not in tmb[test.name].keys():
                        tmb[test.name][metric.name] = {}
                    tbm[test.name][testblock.name][metric.name] = metric
                    tmb[test.name][metric.name][testblock.name] = metric

        print ""
        print "###########"
        print "### tbm ###"
        print "###########"
        print tbm

        print ""
        print "###########"
        print "### tmb ###"
        print "###########"
        print tmb

        # plot tmb
        
        nr_tests = len(tmb.keys())
        nr_metrics_max = 0
        nr_testblocks_max = 0
        for metric_name, metric_data in tmb.items():
            nr_metrics = len(metric_data)
            nr_metrics_max = max(nr_metrics, nr_metrics_max)
            for testblock_name, testblock_data in metric_data.items():
                    nr_testblocks = len(testblock_data)
                    nr_testblocks_max = max(nr_testblocks, nr_testblocks_max)
        
        print "\nplot tmb (rows: %d, cols: %d)"%(nr_tests, nr_metrics_max)
        meanlineprops = dict(linestyle='--', color='purple')
        fig, axs = plt.subplots(nr_tests, nr_metrics_max, sharex=True, figsize=(20, 15)) # FIXME calculate width with nr_testblocks

        for col, test_name in enumerate(tmb):
            axs[col][0].set_ylabel(test_name, rotation=45)
            for row, metric_name in enumerate(tmb[test_name]):
                labels = []
                data = []
                for testblock_name in tmb[test_name][metric_name]:
                    #print testblock_name
                    labels.append(testblock_name)
                    data.append([tmb[test_name][metric_name][testblock_name].data.data])
                title = metric_name
                axs[0][row].set_title(title)
                axs[col][row].boxplot(data, showmeans=True, meanline=True, labels=labels, meanprops=meanlineprops, patch_artist=True)
                axs[col][row].grid(True)

        fig.autofmt_xdate(rotation=45)
        plt.tight_layout()
        plt.show()

        print "DONE plot_fmw"
        return

if __name__ == '__main__':
    # example call could be:
    #   rosrun atf_plotter plot.py plot-metric -m tf_length_translation -tb testblock_circle -t ts0_c0_r0_e0_s0_0 ~/atf_result.txt
    # to get info about the file, this could be helpful:
    #   rosrun atf_plotter plot.py info-structure ~/atf_result.txt

    add_metric = lambda sp: sp.add_argument('--metric', '-m', type=str, dest='metric', required=True, help='TBD')
    add_testblock = lambda sp: sp.add_argument('--testblock', '-tb', type=str, dest='testblock', required=True, help='TBD')
    add_test = lambda sp: sp.add_argument('--test', '-t', type=str, dest='test', required=True, help='TBD')
    add_test_case_ident = lambda sp: sp.add_argument('--testident', '-ti', type=str, dest='test_case_ident', required=True, help='like test name without repetition, e.g. ts0_c0_r0_e0_s0')


    parser = argparse.ArgumentParser(
        conflict_handler='resolve',
        description='WIP CLI for plotting an atf result in different ways',
        epilog='for more information on sub-commands, type SUB-COMMAND --help')


    subparsers = parser.add_subparsers(help='sub-command help TBD', dest='command')


    sub_parser = subparsers.add_parser(
        'plot-metric',
        help='visualize data and groundtruth for a given metric in a given testblock for a given test, e.g. time in '
             'testblock_small from atf_test/ts0_c0_r0_e0_s0_0'
    )
    add_metric(sub_parser)
    add_testblock(sub_parser)
    add_test(sub_parser)


    sub_parser = subparsers.add_parser(
        'plot-b',
        help='visualize data for a all metrics in a given testblock for a given test'
    )
    add_testblock(sub_parser)
    add_test(sub_parser)


    sub_parser = subparsers.add_parser('plot-c', help='visualize data for all metrics in all testblocks in all tests')


    sub_parser = subparsers.add_parser('plot-d', help='visualize aggregated data for all test repetitions for a given test, e.g. atf_test/ts0_c0_r0_e0_s0_0..10')
    add_test_case_ident(sub_parser)

    sub_parser = subparsers.add_parser('plot-fmw', help='fmw test plot')

    sub_parser = subparsers.add_parser('compare-a', help='visualize comparison for a given metric in various testblocks of a given test, e.g. path_length in testblock testblock_small and testblock testblock_large from atf_test/ts0_c0_r0_e0_s0_0')

    sub_parser = subparsers.add_parser('compare-b', help='visualize comparision for all repetitions for a given test, e.g. atf_test/ts0_c0_r0_e0_s0_0..10')

    sub_parser = subparsers.add_parser('visualize-series', help='visualize time series data for a given metric in a given testblock for a given test, e.g. time in testblock_small from atf_test/ts0_c0_r0_e0_s0_0 from all atf_result.txt files')

    sub_parser = subparsers.add_parser('info-structure', help='TBD')


    parser.add_argument('filenames', metavar='filenames', nargs='+',
                        help='merged atf result file (multiple files not yet implemented)')



    #argparse_result = parser.parse_args(['--help'])
    #argparse_result = parser.parse_args(['plot metric', '--help'])
    #argparse_result = parser.parse_args(['plot foo', '--help'])


    #filename = '/tmp/atf_test_app_tf/results_txt/atf_result.txt'
    #filename = '/home/bge/Projekte/atf_data/atf_test_app_tf/results_txt/atf_result.txt'
    #filename = '/home/bge/Projekte/atf_data/atf_test/results_txt/atf_result.txt'

    #filename = '/home/bge/Projekte/atf_data/atf_test_app_navigation__series__atf_result.txt'
    filename = '/home/bge/Projekte/atf_data/atf_test__series__atf_result.txt'

    test_args = [
        [  # 0
            '--help'
        ],
        [  # 1
            'plot-metric',
            '--help'
        ],
        [  # 2
            'plot-metric',
            '-m', 'tf_length_translation',
            '-tb', 'testblock_small',
            '-t', 'ts0_c0_r0_e0_s0_0',
            filename
        ],
        [  # 3
            'plot-b',
            '-tb', 'testblock_small',
            '-t', 'ts0_c0_r0_e0_s0_0',
            filename
        ],
        [  # 4
            'plot-c',
            filename
        ],
        [  # 5
            'plot-d',
            '-ti', 'ts0_c0_r0_e0_s0',
            filename
        ],
        [  # 6
            'plot-fmw',
            filename
        ],
        [  # -1
            'info-structure',
            filename
        ],
    ]

    #argparse_result = parser.parse_args(test_args[5])
    argparse_result = parser.parse_args()


    dp = DemoPlotter()
    print 'loading file...',
    sys.stdout.flush()
    stime = time.time()
    dp.load_atf_result(filename=argparse_result.filenames[0])
    dtime = time.time() - stime
    print 'DONE (took %.3fs)' % (dtime)
    sys.stdout.flush()
    #dp._quicktest()

    #dp.print_structure()


    if argparse_result.command == 'plot-metric':
        dp.plot_data_and_groundtruth_for_given_metric_testblock_test(
            metric=argparse_result.metric,
            testblock=argparse_result.testblock,
            test=argparse_result.test
        )
    elif argparse_result.command == 'plot-b':
        dp.plot_all_metrics_for_given_testblock_test(
            testblock=argparse_result.testblock,
            test=argparse_result.test
        )
    elif argparse_result.command == 'plot-c':
        dp.plot_all_metrics_testblocks_tests()
    elif argparse_result.command == 'plot-d':
        dp.plot_aggregated_data_for_all_test_repetitions_for_given_test_ident(argparse_result.test_case_ident)
    elif argparse_result.command == 'plot-fmw':
        dp.plot_fmw()
    elif argparse_result.command == 'info-structure':
        dp.print_structure()
    else:
        raise NotImplementedError('sub-command <%s> not implemented yet' % argparse_result.command)

