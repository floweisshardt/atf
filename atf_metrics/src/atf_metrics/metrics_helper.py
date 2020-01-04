#!/usr/bin/env python

#import statistics # only available for python >3.4
import numpy

from atf_msgs.msg import DataStamped

def list_from_series(series):
    flat_list = []
    for data in series:
        flat_list.append(data.data)
    return flat_list

def get_min(series):
    result = DataStamped()
    for data in series:
        if data.data < result.data: # '<' takes first accurance for multiple min elements
            result = data
    return result

def get_max(series):
    result = DataStamped()
    for data in series:
        if data.data > result.data: # '>' takes first accurance for multiple max elements
            result = data
    return result

def get_mean(series):
    series_list = list_from_series(series)
    return numpy.mean(series_list)

def get_std(series):
    series_list = list_from_series(series)
    return numpy.std(series_list)
