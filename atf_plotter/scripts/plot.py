#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import matplotlib.style
import matplotlib as mpl
mpl.style.use('classic')

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


import yaml

import pprint
import re

import numpy as np

from matplotlib import cm

from collections import namedtuple

import argparse

import sys

import time

MetricAggregate = namedtuple('MetricAggregate', 'metric_name testblock_name num min max mean median std data_points')


class TimestampBoundData(object):
    def __init__(self, data_dict):
        self._data_dict = data_dict

    @property
    def timestamp(self):
        stamp = self._data_dict['stamp']
        secs = stamp['secs']
        nsecs = stamp['nsecs']
        return secs * nsecs * 1e-9

    @property
    def data(self):
        return self._data_dict['data']


class BaseYamlStructure(object):
    def __init__(self, data_dict, parent=None):
        self._data_dict = data_dict
        self._sublevel_class = dict
        self._sublevel_key = 'results'
        self._sublevel_identifier = 'name'
        self.parent = parent

    @property
    def sublevel_item_list(self):
        return self._data_dict.get(self._sublevel_key, None)

    @property
    def sublevel_names(self):
        return [sub_item.get(self._sublevel_identifier, None) for sub_item in self.sublevel_item_list]

    @property
    def name(self):
        return self._data_dict.get('name', None)

    @property
    def groundtruth_result(self):
        return self._data_dict.get('groundtruth_result', None)

    @property
    def groundtruth_error_message(self):
        return self._data_dict.get('groundtruth_error_message', None)

    def __contains__(self, item):
        return item in self.sublevel_names

    def __getitem__(self, key):
        if key not in self:
            raise KeyError

        if self.sublevel_names.count(key) == 1:
            ridx = self.sublevel_names.index(key)
            sublevel_item = self._sublevel_class(self.sublevel_item_list[ridx], parent=self)
            return sublevel_item
        else:
            sol = [sublevel_item for sublevel_item in self if sublevel_item.name == key]
            return sol
        #print 'count', self.sublevel_names.count(key)

    def __iter__(self):
       for sublevel_item in self.sublevel_item_list:
          yield self._sublevel_class(sublevel_item, parent=self)

    def __len__(self):
        return len(self.sublevel_item_list)



class Metric(BaseYamlStructure):
    def __init__(self, data_dict, **kwargs):
        BaseYamlStructure.__init__(self, data_dict=data_dict, **kwargs)
        self._sublevel_key = 'details'
        self._sublevel_identifier = 'key'

    @property
    def started(self):
        return self._data_dict.get('started', None)

    @property
    def finished(self):
        return self._data_dict.get('finished', None)

    @property
    def data(self):
        return self._data_dict.get('data', None).get('data', None)

    @property
    def series(self):
        raw = self._data_dict.get('series', None)
        if raw is None or not len(raw):
            return raw

        return map(TimestampBoundData, raw)

        #return [(tbd.timestamp, tbd.data) for tbd in map(TimestampBoundData, raw)]

        #return self._data_dict.get('series', None)

    @property
    def series_tuple(self):
        return [(tbd.timestamp, tbd.data) for tbd in self.series]

    @property
    def groundtruth(self):
        return self._data_dict.get('groundtruth', None)

    @property
    def groundtruth_epsilon(self):
        return self._data_dict.get('groundtruth_epsilon', None)

    def get_detail(self, name):
        return self[name]['value']

    @property
    def details(self):
        return self._data_dict.get('details', None)



class TestBlock(BaseYamlStructure):
    def __init__(self, data_dict, **kwargs):
        BaseYamlStructure.__init__(self, data_dict=data_dict, **kwargs)
        self._sublevel_class = Metric



class SingleTest(BaseYamlStructure):
    def __init__(self, data_dict, **kwargs):
        BaseYamlStructure.__init__(self, data_dict=data_dict, **kwargs)
        self._sublevel_class = TestBlock

    @property
    def robot(self):
        return self._data_dict.get('robot', None)

    @property
    def env(self):
        return self._data_dict.get('env', None)

    @property
    def test_config(self):
        return self._data_dict.get('test_config', None)

    @property
    def testblockset(self):
        return self._data_dict.get('testblockset', None)

    @property
    def repetition_number(self):
        num = int(self.name.rsplit('_', 1)[-1])
        return num

    @property
    def test_case_ident(self):
        num = self.name.rsplit('_', 1)[0]
        return num



class AtfResult(BaseYamlStructure):
    def __init__(self, data_dict):
        BaseYamlStructure.__init__(self, data_dict=data_dict)
        self._sublevel_class = SingleTest




class DemoPlotter(object):

    def __init__(self):
        self.atf_result = None


    def load_atf_result(self, filename):

        with open(filename, 'r') as f:
            yaml_data = yaml.load(f, Loader=yaml.CLoader)

        self.atf_result = AtfResult(yaml_data)

    def _quicktest(self):
        assert self.atf_result is not None

        lfr = self.atf_result.sublevel_names[0]

        cre = re.compile('^ts\d+_c\d+_r\d+_e\d+_s\d+_\d+$')
        assert cre.match(lfr) is not None

        assert lfr in self.atf_result


    def get_flat_list_of_metrics(self):
        combined_metrics = list()
        for single_test in self.atf_result:
            for test_block in single_test:
                for metrics in test_block:
                    combined_metrics.append(metrics)
        return combined_metrics

    def group_metric_list_by_repetition_number_for_testcase(self, test_case_ident):
        grouped = dict()

        combined_metrics = self.get_flat_list_of_metrics()

        for metric in combined_metrics:
            if test_case_ident != metric.parent.parent.test_case_ident:
                continue

            rn = metric.parent.parent.repetition_number
            if rn not in grouped:
                grouped[rn] = list()
            grouped[rn].append(metric)

        return grouped



    def aggregate_grouped_metrics(self, grouped_metrics):
        ilen = map(len, grouped_metrics.values())
        assert areAllListValuesEqual(ilen)

        unsuitable_metric_ident_detail_keys = [
            'rot_first_euler',
            'rot_last_euler',
            'lin_first',
            'lin_last',
        ]

        aggregated_metrics = list()
        for aggregate_list in zip(*grouped_metrics.values()):

            metric_ident = zip(*[(
                m.name,
                m.parent.name,
                '/'.join(['%r' % d['value'] for d in m.details if d['key'] not in unsuitable_metric_ident_detail_keys])
            ) for m in aggregate_list])

            allEqual = areAllListValuesEqual(map(areAllListValuesEqual, metric_ident))
            assert allEqual and True

            metric_ident_reduced = [mi[0] for mi in metric_ident]

            metric_values = zip(*[(
                m.data,
                m.groundtruth,
                m.groundtruth_epsilon,
                m.groundtruth_result
            ) for m in aggregate_list])

            metric_data = metric_values[0]

            aggregate = MetricAggregate(
                metric_name=metric_ident_reduced[0],
                testblock_name=metric_ident_reduced[1],
                num=len(metric_data),
                min=min(metric_data),
                max=max(metric_data),
                mean=np.mean(metric_data),
                median=np.median(metric_data),
                std=np.std(metric_data),
                data_points=metric_data
            )

            aggregated_metrics.append(aggregate)

        return aggregated_metrics


    def plot_data_and_groundtruth_for_given_metric_testblock_test(self, metric, testblock, test):
        single_test = self.atf_result[test]
        assert isinstance(single_test, SingleTest)

        print 'single_test.repetition_number', single_test.repetition_number

        test_block = single_test[testblock]
        assert isinstance(test_block, TestBlock)

        metrics = test_block[metric]
        #assert isinstance(metric, Metric)


        fig, ax = plt.subplots()

        width = 0.2

        for idx, metric in enumerate(metrics):
            #print metric.details
            #label = metric['measured_frame']['value']
            #label = '%s/%s/%s' % (single_test.name, test_block.name, metric.get_detail('measured_frame'))

            label = build_tmp_label(metric)
            rects = ax.bar(idx - width / 2.0, metric.data, width, label=label)
            autolabel_ax_bar(ax, rects)

            print metric.data
            print metric.groundtruth
            print metric.groundtruth_epsilon
            print

        ax.legend()
        plt.show()


    def plot_all_metrics_for_given_testblock_test(self, testblock, test):
        single_test = self.atf_result[test]
        assert isinstance(single_test, SingleTest)

        test_block = single_test[testblock]
        assert isinstance(test_block, TestBlock)


        fig, ax = plt.subplots()

        width = 0.2

        clut = cm._generate_cmap('jet', len(test_block))
        for idx, metric in enumerate(test_block):
            label = build_tmp_label(metric)
            rects = ax.bar(idx - width / 2.0, metric.data, width, label=label, color=clut(idx))
            autolabel_ax_bar(ax, rects)

            print metric.data
            print metric.groundtruth
            print metric.groundtruth_epsilon
            print

        ax.legend()
        plt.show()


    def plot_all_metrics_testblocks_tests(self):
        combined_metrics = self.get_flat_list_of_metrics()

        fig, ax = plt.subplots()
        width = 0.75

        clut = cm._generate_cmap('jet', len(combined_metrics))

        for idx, metric in enumerate(combined_metrics):
            label = build_tmp_label(metric)
            print label
            print ' '*7, metric.details

            rects = ax.bar(idx - width / 2.0, metric.data, width, label=label, color=clut(idx))
            autolabel_ax_bar(ax, rects)

        ax.legend()
        plt.show()


    def plot_aggregated_data_for_all_test_repetitions_for_given_test_ident(self, test_case_ident):
        grm = self.group_metric_list_by_repetition_number_for_testcase(test_case_ident)
        amr = self.aggregate_grouped_metrics(grm)

        data = [am.data_points for am in amr]
        labels = [am.metric_name for am in amr]
        category = [am.testblock_name for am in amr]

        fig, ax = plt.subplots()

        meanlineprops = dict(linestyle='--', color='purple')
        hdls1 = ax.boxplot(data, showmeans=True, meanline=True, meanprops=meanlineprops, labels=labels, patch_artist=True)

        uniqe_category_list = list(set(category))
        clut = cm._generate_cmap('Dark2', len(uniqe_category_list))
        colors = [clut(uniqe_category_list.index(cat)) for cat in category]

        #colors = [clut(i) for i in range(50)]

        for idx, label in enumerate(ax.get_xmajorticklabels()):
            label.set_rotation(90)
            label.set_color(colors[idx])
            print idx, label#, colors[idx]
            #label.set_horizontalalignment("right")

        # fill with colors



        #colors = ['blue', 'pink', 'lightblue', 'lightgreen']*5
        for patch, color in zip(hdls1['boxes'], colors):
            patch.set_facecolor(color)

        custom_lines = [Line2D([0], [0], color=c, lw=4) for c in set(colors)]
        ax.legend(custom_lines, uniqe_category_list, ncol=2)


        ax.grid(True)


        plt.tight_layout()

        #plt.xticks(range(len(labels)+1), labels, rotation='vertical')
        #plt.subplots_adjust(bottom=0.15)


        #ax.boxplot(amr[7].data_points, showmeans=True, meanline=True)
        #ax.violinplot(amr[7].data_points, [2], points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True, bw_method='silverman')

        #for idx, ma in enumerate(amr):
        #    try:
        #        ax.violinplot(ma.data_points, [idx], points=60, widths=0.7, showmeans=False, showextrema=True, showmedians=True, bw_method=0.5)
        #    except:
        #        pass
            #break


        #ax.legend()
        plt.show()

    def print_structure(self):
        for test in self.atf_result:
            print test.name
            for testblock in test:
                print ' '*3, testblock.name
                old_name = None
                metric_names = testblock.sublevel_names
                series_len = 0
                for metric in testblock:
                    series_len += len(metric.series)
                    if old_name == metric.name:
                        continue
                    old_name = metric.name
                    print ' '*7, '%dx' % metric_names.count(metric.name), metric.name, '   (series available, length %d)' % series_len if series_len else ''



def build_tmp_label(metric):
    #label = '%s/%s/%s/%s' % (metric.parent.parent.name, metric.parent.name, metric.name, metric.get_detail('measured_frame'))
    label = '%s/%s/%s' % (metric.parent.parent.name, metric.parent.name, metric.name)
    try:
        label += '/%s' % metric.get_detail('measured_frame')
    except KeyError:
        pass

    return label


def areAllListValuesEqual(lst):
    return not lst or lst.count(lst[0]) == len(lst)


def autolabel_ax_bar(ax, rects):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')


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
    elif argparse_result.command == 'info-structure':
        dp.print_structure()
    else:
        raise NotImplementedError('sub-command <%s> not implemented yet' % argparse_result.command)

