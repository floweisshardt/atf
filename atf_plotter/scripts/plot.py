#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import matplotlib.style
import matplotlib as mpl
mpl.style.use('classic')

import fnmatch

from atf_msgs.msg import AtfResult, MetricResult

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

class AtfPlotter(object):

    def __init__(self):
        self.atf_result = None


    def load_atf_result(self, filename):
        
        bag = rosbag.Bag(filename)
        assert bag.get_message_count() == 1 # check if only one message is in the bag file
            
        for topic, msg, t in bag.read_messages():
            #print type(msg), type(AtfResult()), isinstance(msg, AtfResult) # check if message is an AtfResult message
            #assert isinstance(msg, AtfResult) # check if message is an AtfResult message
            if topic == "atf_result":
                self.atf_result = msg
            else:
                print "ERROR: invalid topic name in bag. ATF only expects topic 'atf_result'"
            bag.close()
            break

    def get_sorted_plot_dicts(self, atf_result, filter_tests, filter_testblocks, filter_metrics):
        tbm = {}
        tmb = {}
        bmt = {}
        mbt = {}

        for test in atf_result.results:
            #print test.name
            if len(filter_tests) != 0 and not fnmatch.fnmatch(test.name, filter_tests):
                continue

            for testblock in test.results:
                #print "  -", testblock.name
                if len(filter_testblocks) != 0 and not fnmatch.fnmatch(testblock.name, filter_testblocks):
                    continue

                for metric in testblock.results:
                    #print "    -", metric.name
                    split_name = metric.name.split("::")
                    if len(filter_metrics) != 0 and not fnmatch.fnmatch(metric.name, filter_metrics) and not fnmatch.fnmatch(split_name[0],filter_metrics):
                        continue

                    # tbm
                    if test.name                 not in tbm.keys():
                        tbm[test.name] = {}
                    if testblock.name            not in tbm[test.name].keys():
                        tbm[test.name][testblock.name] = {}
                    tbm[test.name][testblock.name][metric.name] = metric

                    # tmb
                    if test.name                 not in tmb.keys():
                        tmb[test.name] = {}
                    if metric.name               not in tmb[test.name].keys():
                        tmb[test.name][metric.name] = {}
                    tmb[test.name][metric.name][testblock.name] = metric

                    # bmt
                    if testblock.name            not in bmt.keys():
                        bmt[testblock.name] = {}
                    if metric.name               not in bmt[testblock.name].keys():
                        bmt[testblock.name][metric.name] = {}
                    bmt[testblock.name][metric.name][test.name] = metric

                    # mbt
                    if metric.name            not in mbt.keys():
                        mbt[metric.name] = {}
                    if testblock.name         not in mbt[metric.name].keys():
                        mbt[metric.name][testblock.name] = {}
                    mbt[metric.name][testblock.name][test.name] = metric

        ret = {}
        ret['tbm'] = tbm
        ret['tmb'] = tmb
        ret['bmt'] = bmt
        ret['mbt'] = mbt
        return ret

    def print_structure(self):
        for test in self.atf_result.results:
            print test.name
            for testblock in test.results:
                print "  -", testblock.name
                for metric in testblock.results:
                    print "    -", metric.name

    def plot_benchmark(self, style, sharey, hide_groundtruth, hide_min_max, filter_tests, filter_testblocks, filter_metrics):

        sorted_atf_results = self.get_sorted_plot_dicts(self.atf_result, filter_tests, filter_testblocks, filter_metrics)

        if style not in sorted_atf_results.keys():
            print "ERROR: style '%s' not implemented"%style
            return
        plot_dict = sorted_atf_results[style]

        rows = plot_dict.keys()
        cols = []
        plots = []
        nr_unique_plots = 0
        for row in rows:
            cols_tmp = plot_dict[row].keys()
            for col in cols_tmp:
                if col not in cols:
                    cols.append(col)
                    for plot in plot_dict[row][col].keys():
                        if plot not in plots:
                            plots.append(plot)

        # sort alphabetically
        rows.sort()
        cols.sort()
        plots.sort()

        print "\nplotting in style '%s' (rows: %d, cols: %d, plots: %d)"%(style, len(rows), len(cols), len(plots))
        meanlineprops = dict(linestyle='--', color='purple')

        fig, axs = plt.subplots(len(rows), len(cols), squeeze=False, sharex=True, sharey=sharey, figsize=(20, 15)) # FIXME calculate width with nr_testblocks

        # always make this a numpy 2D matrix to access rows and cols correclty if len(rows)=1 or len(cols)=1
        #axs = np.atleast_2d(axs) 
        #axs = axs.reshape(len(rows), len(cols))
        # --> not needed anymore due to squeeze=False

        y_min_overall = 0
        y_max_overall = 0

        for row in rows:
            #print "\nrow=", row
            
            
            for col in cols:
                #print "  col=", col

                x = np.arange(len(plots))
                ax = axs[rows.index(row)][cols.index(col)]

                # format x axis
                ax.set_xticks(x)
                ax.set_xticklabels(plots)
                ax.set_xlim(-1, len(plots))

                # format y axis
                ax.autoscale(enable=True, axis='y', tight=False)
                ax.margins(y=0.2) # make it a little bigger than the min/max values

                # only set title for upper row and ylabel for left col
                if rows.index(row) == 0:
                    ax.set_title(col)
                if cols.index(col) == 0:
                    ax.set_ylabel(row, rotation=90)

                for plot in plots:
                    #print "    plot=", plot
                    try:
                        metric_result = plot_dict[row][col][plot]
                    except KeyError:
                        #print "skip", row, col, plot
                        continue

                    ax.grid(True)
                    nr_unique_plots += 1

                    data = metric_result.data.data
                    lower = metric_result.groundtruth.data - metric_result.groundtruth.epsilon
                    upper = metric_result.groundtruth.data + metric_result.groundtruth.epsilon
                    y_min = min(-0.1, data, lower, upper)
                    y_max = max(data, lower, upper)

                    yerr = [[0], [0]]
                    if not hide_groundtruth and metric_result.groundtruth.available:
                        yerr = [[data - lower], [upper - data]]
                    ax.errorbar(plots.index(plot), data, yerr=yerr, fmt='D', markersize=12)

                    # plot min and max
                    if not hide_min_max and metric_result.mode == MetricResult.SPAN:
                        ax.plot(plots.index(plot), metric_result.min.data, '^', markersize=8)
                        ax.plot(plots.index(plot), metric_result.max.data, 'v', markersize=8)

                    # plot a non-visible zero for y-axis scaling
                    ax.plot(plots.index(plot), 0, '')

        fig.autofmt_xdate(rotation=45)
        plt.tight_layout()

        title = "ATF Result for %s\ntotal # of tests: %d\ntotal # of plots: %d"%("PACKAGE", len(self.atf_result.results), nr_unique_plots)   # replace PACKAGE with self.atf_result.package (needs to be added to message first)
        st = fig.suptitle(title, fontsize="x-large")
        # shift subplots down:
        st.set_y(0.95)
        fig.subplots_adjust(top=0.85)

        fig.savefig("/tmp/test.png")
        plt.show()
        return

if __name__ == '__main__':
    # example call could be:
    #   rosrun atf_plotter plot.py plot-benchmark /tmp/atf_test/results_txt/atf_result.bag
    #   rosrun atf_plotter plot.py plot-benchmark -m publish_rate /tmp/atf_test/results_txt/atf_result.bag
    #   rosrun atf_plotter plot.py plot-benchmark -m publish_rate -tb testblock_small -t ts0_c0_r0_e0_s0_0 /tmp/atf_test/results_txt/atf_result.bag
    # to get info about the file, this could be helpful:
    #   rosrun atf_plotter plot.py info-structure ~/atf_result.bag

    add_test = lambda sp: sp.add_argument('--test', '-t', type=str, dest='test', default="", help='TBD')
    add_test_case_ident = lambda sp: sp.add_argument('--testident', '-ti', type=str, dest='test_case_ident', required=True, help='like test name without repetition, e.g. ts0_c0_r0_e0_s0')
    add_testblock = lambda sp: sp.add_argument('--testblock', '-tb', type=str, dest='testblock', default="", help='TBD')
    add_metric = lambda sp: sp.add_argument('--metric', '-m', type=str, dest='metric', default="", help='TBD')
    add_style = lambda sp: sp.add_argument('--style', '-s', type=str, dest='style', default='bmt', help='style, e.g. bmt (default) tmb, tmb, bmt, ...')
    add_sharey = lambda sp: sp.add_argument('--sharey', type=str, dest='sharey', default='none', help='share y axis for all plots. [none|all|row|col], default = none')
    add_hide_groundtruth = lambda sp: sp.add_argument('--hide-groundtruth', dest='hide_groundtruth', action='store_true', help='hide groundtruth values even if goundtruth.available=True. default = False')
    add_hide_min_max = lambda sp: sp.add_argument('--hide-min-max', dest='hide_min_max', action='store_true', help='hide min and max values even if mode=SPAN. default = False')

    parser = argparse.ArgumentParser(
        conflict_handler='resolve',
        description='WIP CLI for plotting an atf result in different ways',
        epilog='for more information on sub-commands, type SUB-COMMAND --help')


    subparsers = parser.add_subparsers(help='sub-command help TBD', dest='command')

    sub_parser = subparsers.add_parser('plot-benchmark', help='Visualize benchmark plots. Can be filtered via tests, testblocks and metrics. Style is a combination (order) of _t_est, test_b_lock and _m_etric')
    add_style(sub_parser)
    add_sharey(sub_parser)
    add_hide_groundtruth(sub_parser)
    add_hide_min_max(sub_parser)
    add_test(sub_parser)
    add_testblock(sub_parser)
    add_metric(sub_parser)

    sub_parser = subparsers.add_parser('plot-series', help='visualize time series data for a given metric in a given testblock for a given test, e.g. time in testblock_small from atf_test/ts0_c0_r0_e0_s0_0 from all atf_result.txt files')

    sub_parser = subparsers.add_parser('info-structure', help='TBD')


    parser.add_argument('filenames', metavar='filenames', nargs='+',
                        help='merged atf result file (multiple files not yet implemented)')
    argparse_result = parser.parse_args()


    atf_plotter = AtfPlotter()
    print 'loading file...',
    sys.stdout.flush()
    stime = time.time()
    atf_plotter.load_atf_result(filename=argparse_result.filenames[0])
    dtime = time.time() - stime
    print 'DONE (took %.3fs)' % (dtime)
    sys.stdout.flush()

    #atf_plotter.print_structure()

    if argparse_result.command == 'plot-benchmark':
        atf_plotter.plot_benchmark(
            style =                 argparse_result.style,
            sharey =                argparse_result.sharey,
            hide_groundtruth =      argparse_result.hide_groundtruth,
            hide_min_max =          argparse_result.hide_min_max,
            filter_tests =          argparse_result.test,
            filter_testblocks =     argparse_result.testblock,
            filter_metrics =        argparse_result.metric)
    elif argparse_result.command == 'plot-series':
        atf_plotter.plot_series(
            filter_tests =          argparse_result.test,
            filter_testblocks =     argparse_result.testblock,
            filter_metrics =        argparse_result.metric
        )
    elif argparse_result.command == 'info-structure':
        atf_plotter.print_structure()
    else:
        raise NotImplementedError('sub-command <%s> not implemented yet' % argparse_result.command)

