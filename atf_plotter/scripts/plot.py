#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import yaml
import pprint
import re

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from matplotlib import cm

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
        return self._data_dict.get('data', None)

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



class AtfResult(BaseYamlStructure):
    def __init__(self, data_dict):
        BaseYamlStructure.__init__(self, data_dict=data_dict)
        self._sublevel_class = SingleTest




class DemoPlotter(object):

    def __init__(self):
        self.atf_result = None


    def load_atf_result(self, filename):

        with open(filename, 'r') as f:
            yaml_data = yaml.safe_load(f)

        self.atf_result = AtfResult(yaml_data)

    def _quicktest(self):
        assert self.atf_result is not None

        lfr = self.atf_result.sublevel_names[0]

        cre = re.compile('^ts\d+_c\d+_r\d+_e\d+_s\d+_\d+$')
        assert cre.match(lfr) is not None

        assert lfr in self.atf_result




    def plot_data_and_groundtruth_for_given_metric_testblock_test(self, metric, testblock, test):
        single_test = self.atf_result[test]
        assert isinstance(single_test, SingleTest)

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

        combined_metrics = list()
        for single_test in self.atf_result:
            for test_block in single_test:
                for metrics in test_block:
                    combined_metrics.append(metrics)

        fig, ax = plt.subplots()
        width = 0.75

        clut = cm._generate_cmap('jet', len(combined_metrics))

        for idx, metric in enumerate(combined_metrics):
            label = build_tmp_label(metric)
            print label
            print ' '*7, metric.details

            rects = ax.bar(idx - width / 2.0, metric.data, width, label=label, color=clut(idx))
            autolabel_ax_bar(ax, rects)

        #ax.legend()
        plt.show()


def build_tmp_label(metric):
    label = metric['measured_frame']['value']
    label = '%s/%s/%s/%s' % (metric.parent.parent.name, metric.parent.name, metric.name, metric.get_detail('measured_frame'))
    return label



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
    #filename = '/tmp/atf_test_app_tf/results_txt/atf_result.txt'
    filename = '/home/bge/Projekte/atf_data/atf_test_app_tf/results_txt/atf_result.txt'

    dp = DemoPlotter()
    dp.load_atf_result(filename=filename)
    dp._quicktest()

    '''
    dp.plot_data_and_groundtruth_for_given_metric_testblock_test(
        metric='tf_length_translation',
        testblock='testblock_circle',
        test='ts0_c0_r0_e0_s0_0'
    )#'''

    '''
    dp.plot_all_metrics_for_given_testblock_test(
        testblock='testblock_circle',
        test='ts0_c0_r0_e0_s0_0'
    )#'''

    dp.plot_all_metrics_testblocks_tests()

    #with open(filename, 'r') as f:
    #    data = yaml.safe_load(f)

    #ft = AtfResult(data)

    #lfr = ft.sublevel_names[1]

    #print lfr in ft

    #print ft[lfr]

    #for r in ft:
    #    print r





    #pprint.pprint(data)
