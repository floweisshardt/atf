#!/usr/bin/env python
import copy
import math
import os
import rospy
import sys
import tf
import tf2_py
import tf2_ros

from atf_core import ATFAnalyserError, ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped
from atf_metrics import metrics_helper

class CalculateTfDistanceTranslationParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(self, testblock_name, metric_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metric_type = "tf_distance_translation"

        split_name = metric_name.split("::")
        if len(split_name) != 2:
            raise ATFConfigurationError("no valid metric name for metric '%s' in testblock '%s'" %(metric_name, testblock_name))
        if split_name[0] != metric_type:
            raise ATFConfigurationError("called invalid metric handle for metric '%s' in testblock '%s'." %(metric_name, testblock_name))

        if type(params) is not dict:
            rospy.logerr("metric config not a dictionary")
            raise ATFConfigurationError("no valid metric configuration for metric '%s' in testblock '%s': %s" %(metric_name, testblock_name, str(params)))

        # check for optional parameters
        groundtruth = Groundtruth()
        try:
            groundtruth.data = params["groundtruth"]["data"]
            groundtruth.epsilon = params["groundtruth"]["epsilon"]
            groundtruth.available = True
        except (TypeError, KeyError):
            groundtruth.data = 0
            groundtruth.epsilon = 0
            groundtruth.available = False
        try:
            mode = params["mode"]
        except (TypeError, KeyError):
            mode = MetricResult.SNAP
        try:
            series_mode = params["series_mode"]
        except (TypeError, KeyError):
            series_mode = None

        return CalculateTfDistanceTranslation(metric_name, testblock_name, params["topics"], params["root_frame"], params["measured_frame"], groundtruth, mode, series_mode)

class CalculateTfDistanceTranslation:
    def __init__(self, name, testblock_name, topics, root_frame, measured_frame, groundtruth, mode, series_mode):
        """
        Class for calculating the distance covered by the given frame in relation to a given root frame.
        The tf data is sent over the tf topics given in the test_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """
        self.name = name
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.testblock_name = testblock_name
        self.topics = topics
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.data = DataStamped()

        self.t = tf.Transformer(True, rospy.Duration(10.0))

    def start(self, status):
        self.active = True
        self.started = True

    def stop(self, status):
        self.active = False
        self.finished = True

    def pause(self, status):
        self.active = False

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
            data = self.get_data(t)
            if data == None:
                return
            self.data.stamp = t
            self.data.data = round(data, 6)
            self.series.append(copy.deepcopy(self.data))  # FIXME handle fixed rates

    def get_data(self, t):
        try:
            sys.stdout = open(os.devnull, 'w') # supress stdout
            (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
        except tf2_ros.LookupException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except tf2_py.ExtrapolationException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except tf2_py.ConnectivityException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except Exception as e:
            sys.stdout = sys.__stdout__  # restore stdout
            print "general exeption in metric '%s':"%self.name, type(e), e
            return None
        sys.stdout = sys.__stdout__  # restore stdout

        # This calculates the tf distance from the first transform (root_frame to measured_frame) to the last transform (root_frame to measured_frame)
        #self.data.data = round(sum([(fl - ll)**2 for fl, ll in zip(lin_first, lin_last)])**0.5, 9)

        # This calculates the tf distance between root_frame and measured_frame at the end of the testblock
        distance = sum(axis**2 for axis in trans)**0.5

        return distance

    def get_topics(self):
        return self.topics

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.mode = self.mode
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.series = []
        metric_result.groundtruth.available = self.groundtruth.available
        metric_result.groundtruth.data = self.groundtruth.data
        metric_result.groundtruth.epsilon = self.groundtruth.epsilon

        if not self.started:
            error_message = "testblock %s never started"%self.testblock_name
            #print error_message
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = error_message
            return metric_result

        if not self.finished:
            error_message = "testblock %s never stopped"%self.testblock_name
            #print error_message
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = error_message
            return metric_result

        # check if result is available
        if len(self.series) == 0:
            # let the analyzer know that this test failed
            metric_result.groundtruth.result = False
            metric_result.groundtruth.error_message = "testblock %s stopped without result"%self.testblock_name
            return metric_result

        # at this point we're sure that any result is available

        # calculate metric data
        if self.series_mode != None:
            metric_result.series = self.series
        if metric_result.mode == MetricResult.SNAP:
            metric_result.data = self.series[-1]                           # take last element from self.series for data and stamp
            metric_result.min = metric_result.data
            metric_result.max = metric_result.data
            metric_result.mean = metric_result.data.data
            metric_result.std = 0.0
        elif metric_result.mode == MetricResult.SPAN_MEAN:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data.data = metric_result.mean                   # take mean for data
            metric_result.data.stamp = self.series[-1].stamp               # take stamp from last element in self.series for stamp
        elif metric_result.mode == MetricResult.SPAN_MIN:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.min
        elif metric_result.mode == MetricResult.SPAN_ABSMIN:
            metric_result.min = metrics_helper.get_absmin(self.series)
            metric_result.max = metrics_helper.get_absmax(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.min
        elif metric_result.mode == MetricResult.SPAN_MAX:
            metric_result.min = metrics_helper.get_min(self.series)
            metric_result.max = metrics_helper.get_max(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.max
        elif metric_result.mode == MetricResult.SPAN_ABSMAX:
            metric_result.min = metrics_helper.get_absmin(self.series)
            metric_result.max = metrics_helper.get_absmax(self.series)
            metric_result.mean = metrics_helper.get_mean(self.series)
            metric_result.std = metrics_helper.get_std(self.series)
            metric_result.data = metric_result.max
        else: # invalid mode
            raise ATFAnalyserError("Analysing failed, invalid mode '%s' for metric '%s'."%(metric_result.mode, metric_result.name))

        # fill details as KeyValue messages
        details = []
        details.append(KeyValue("root_frame", self.root_frame))
        details.append(KeyValue("measured_frame", self.measured_frame))
        metric_result.details = details

        # evaluate metric data
        if metric_result.groundtruth.available: # groundtruth available
            if math.fabs(metric_result.groundtruth.data - metric_result.data.data) <= metric_result.groundtruth.epsilon:
                metric_result.groundtruth.result = True
                metric_result.groundtruth.error_message = "all OK"
            else:
                metric_result.groundtruth.result = False
                metric_result.groundtruth.error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth.data, metric_result.groundtruth.epsilon)

        else: # groundtruth not available
            metric_result.groundtruth.result = True
            metric_result.groundtruth.error_message = "all OK (no groundtruth available)"

        return metric_result
