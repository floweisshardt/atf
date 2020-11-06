#!/usr/bin/env python

#import statistics # only available for python >3.4
import numpy

from atf_metrics.error import ATFAnalyserError
from atf_msgs.msg import DataStamped, TestblockStatus, MetricResult

def list_from_series(series):
    flat_list = []
    for data in series:
        flat_list.append(data.data)
    return flat_list

def get_min(series):
    result = None
    for data in series:
        if result == None:
            result = data
        elif data.data < result.data: # '<' takes first accurance for multiple min elements
            result = data
    return result

def get_absmin(series):
    result = None
    for data in series:
        data.data = abs(data.data)
        if result == None:
            result = data
        elif data.data < result.data: # '<' takes first accurance for multiple min elements
            result = data
    return result

def get_max(series):
    result = None
    for data in series:
        if result == None:
            result = data
        elif data.data > result.data: # '>' takes first accurance for multiple max elements
            result = data
    return result

def get_absmax(series):
    result = None
    for data in series:
        data.data = abs(data.data)
        if result == None:
            result = data
        elif data.data > result.data: # '>' takes first accurance for multiple max elements
            result = data
    return result

def get_mean(series):
    series_list = list_from_series(series)
    return numpy.mean(series_list)

def get_std(series):
    series_list = list_from_series(series)
    return numpy.std(series_list)

def calculate_metric_data(metric_name, mode, series):
    # return data, min, max, mean, std
    data = None
    min = None
    max = None
    mean = None
    std = None
    if mode == MetricResult.SNAP:
        data = series[-1]                           # take last element from series for data and stamp
        min = data
        max = data
        mean = data.data
        std = 0.0
    elif mode == MetricResult.SPAN_MEAN:
        min = get_min(series)
        max = get_max(series)
        mean = get_mean(series)
        std = get_std(series)
        data = DataStamped()
        data.data = mean                            # take mean for data
        data.stamp = series[-1].stamp               # take stamp from last element in series for stamp
    elif mode == MetricResult.SPAN_MIN:
        min = get_min(series)
        max = get_max(series)
        mean = get_mean(series)
        std = get_std(series)
        data = min
    elif mode == MetricResult.SPAN_ABSMIN:
        min = get_absmin(series)
        max = get_absmax(series)
        mean = get_mean(series)
        std = get_std(series)
        data = min
    elif mode == MetricResult.SPAN_MAX:
        min = get_min(series)
        max = get_max(series)
        mean = get_mean(series)
        std = get_std(series)
        data = max
    elif mode == MetricResult.SPAN_ABSMAX:
        min = get_absmin(series)
        max = get_absmax(series)
        mean = get_mean(series)
        std = get_std(series)
        data = max
    else: # invalid mode
        raise ATFAnalyserError("Analysing failed, invalid mode '%s' for metric '%s'."%(mode, metric_name))

    res = [data, min, max, mean, std]
    
    # verify that all field are set
    if None in res:
        ATFAnalyserError("Analysing failed, invalid data for metric '%s'."%(metric_name))

    return res

def extract_error_message(status):
    if status.status == TestblockStatus.SUCCEEDED:
        return "All OK"
    elif status.status == TestblockStatus.INACTIVE:
        return "testblock %s never started. status = %d"%(status.name, status.status)
    elif status.status == TestblockStatus.ERROR:
        return "testblock %s errored. status = %d"%(status.name, status.status)
    else: # not in a terminal state
        return "testblock %s never stopped. status = %d"%(status.name, status.status)
    return "something went wrong in testblock %s"%(status.name)