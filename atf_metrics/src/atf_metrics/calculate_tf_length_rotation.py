#!/usr/bin/env python
import copy
import math
import os
import rospy
import sys
import tf
import tf2_py
import tf2_ros

from tf import transformations

from atf_core import ATFAnalyserError
from atf_msgs.msg import MetricResult, KeyValue, DataStamped
from atf_metrics import metrics_helper

class CalculateTfLengthRotationParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            # check for optional parameters
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                groundtruth = None
                groundtruth_epsilon = None
            try:
                series_mode = metric["series_mode"]
            except (TypeError, KeyError):
                series_mode = None
            metrics.append(CalculateTfLengthRotation(metric["topics"], metric["root_frame"], metric["measured_frame"], groundtruth, groundtruth_epsilon, series_mode))
        return metrics

class CalculateTfLengthRotation:
    def __init__(self, topics, root_frame, measured_frame, groundtruth, groundtruth_epsilon, series_mode):
        """
        Class for calculating the distance covered by the given frame in relation to a given root frame.
        The tf data is sent over the tf topics given in the test_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """
        self.name = 'tf_length_rotation'
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.topics = topics
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.series_mode = series_mode
        self.series = []
        self.data = DataStamped()
        self.first_value = True
        self.trans_old = []
        self.rot_old = []

        self.t = tf.Transformer(True, rospy.Duration(10.0))

    def start(self, status):
        self.active = True
        self.started = True

    def stop(self, status):
        self.active = False
        self.finished = True

    def pause(self, status):
        self.active = False
        self.first_value = True

    def purge(self, status):
        pass

    def update(self, topic, msg, t):
        # make sure we're handling a TFMessage (from /tf or /tf_static)
        # TODO check type instead of topic names
        if topic in self.topics:
            for transform in msg.transforms:
                self.t.setTransform(transform)

        # get data if testblock is active
        if self.active:
            self.data.stamp = t
            self.data.data += round(self.get_path_increment(),6)
            self.series.append(copy.deepcopy(self.data))  # FIXME handle fixed rates

    def get_path_increment(self):
        path_increment = 0.0
        try:
            sys.stdout = open(os.devnull, 'w') # supress stdout
            (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
        except tf2_ros.LookupException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return path_increment
        except tf2_py.ExtrapolationException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return path_increment
        except tf2_py.ConnectivityException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return path_increment
        except Exception as e:
            sys.stdout = sys.__stdout__  # restore stdout
            print "general exeption in metric '%s':"%self.name, type(e), e
            return path_increment
        sys.stdout = sys.__stdout__  # restore stdout

        if self.first_value:
            self.trans_old = trans
            self.rot_old = rot
            self.first_value = False
            return path_increment

        diff_q = transformations.quaternion_multiply(rot, transformations.quaternion_conjugate(self.rot_old))
        diff_e = transformations.euler_from_quaternion(diff_q)
        path_increment = sum([axis**2 for axis in diff_e])**0.5

        self.trans_old = trans
        self.rot_old = rot

        return path_increment

    def get_topics(self):
        return self.topics

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.series = []
        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            if self.series_mode != None:
                metric_result.series = self.series
            metric_result.data = self.series[-1] # take last element from self.series
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)

            # fill details as KeyValue messages
            details = []
            details.append(KeyValue("root_frame", self.root_frame))
            details.append(KeyValue("measured_frame", self.measured_frame))
            metric_result.details = details

            # evaluate metric data
            if metric_result.groundtruth == None and metric_result.groundtruth_epsilon == None: # no groundtruth given
                metric_result.groundtruth_result = True
                metric_result.groundtruth_error_message = "all OK (no groundtruth available)"
            elif metric_result.data != None and metric_result.groundtruth != None and metric_result.groundtruth_epsilon != None:
                if math.fabs(metric_result.groundtruth - metric_result.data.data) <= metric_result.groundtruth_epsilon:
                    metric_result.groundtruth_result = True
                    metric_result.groundtruth_error_message = "all OK"
                else:
                    metric_result.groundtruth_result = False
                    metric_result.groundtruth_error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth, metric_result.groundtruth_epsilon)
            else:
                metric_result.groundtruth_result = False
                metric_result.groundtruth_error_message = "metric evaluation failed"

        if metric_result.data == None:
            metric_result.groundtruth_result = False
            metric_result.groundtruth_error_message = "no result"

        if metric_result.groundtruth_result == None:
            raise ATFAnalyserError("Analysing failed, metric result is None for metric '%s'."%metric_result.name)

        return metric_result
