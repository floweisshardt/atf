#!/usr/bin/env python
import math
import os
import rospy
import sys
import tf
import tf2_ros

from atf_metrics.error import ATFConfigurationError
from atf_msgs.msg import MetricResult, Groundtruth, KeyValue, DataStamped, TestblockStatus
from atf_metrics import metrics_helper

class CalculateTfJerkTranslationParamHandler:
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
        metric_type = "tf_jerk_translation"
        unit = "m/s^3"

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
            mode = MetricResult.SPAN_ABSMAX
        try:
            series_mode = params["series_mode"]
        except (TypeError, KeyError):
            series_mode = None

        return CalculateTfJerkTranslation(metric_name, testblock_name, params["topics"], params["root_frame"], params["measured_frame"], groundtruth, mode, series_mode, unit)

class CalculateTfJerkTranslation:
    def __init__(self, name, testblock_name, topics, root_frame, measured_frame, groundtruth, mode, series_mode, unit):
        """
        Class for calculating the jerk by the given frame in relation to a given root frame.
        The tf data is sent over the tf topics given in the test_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The jerk will be measured in relation to the root_frame.
        :type  measured_frame: string
        """
        self.name = name
        self.testblock_name = testblock_name
        self.status = TestblockStatus()
        self.status.name = testblock_name
        self.groundtruth = groundtruth
        self.mode = mode
        self.series_mode = series_mode
        self.series = []
        self.unit = unit

        self.topics = topics
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.trans_old = []
        self.rot_old = []
        self.time_old = None
        self.velocity_old = None
        self.acceleration_old = None

        self.t = tf.Transformer(True, rospy.Duration(10.0))

    def start(self, status):
        self.status = status

    def stop(self, status):
        self.status = status

    def pause(self, status):
        pass

    def purge(self, status):
        pass

    def update(self, topic, msg, t):
        # make sure we're handling a TFMessage (from /tf or /tf_static)
        # TODO check type instead of topic names
        if topic in self.topics:
            for transform in msg.transforms:
                self.t.setTransform(transform)

        # get data if testblock is active
        if self.status.status == TestblockStatus.ACTIVE:
            raw_data = self.get_data(t)
            if raw_data == None:
                return
            data = DataStamped()
            data.stamp = t
            data.data = round(raw_data, 6)
            self.series.append(data)  # FIXME handle fixed rates

    def get_data(self, t):
        try:
            sys.stdout = open(os.devnull, 'w') # supress stdout
            (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
        except tf.LookupException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except tf.ExtrapolationException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except tf.ConnectivityException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric '%s' %s %s"%(self.name, type(e), e)
            return None
        except Exception as e:
            sys.stdout = sys.__stdout__  # restore stdout
            print("general exeption in metric '%s':"%self.name, type(e), e)
            return None
        sys.stdout = sys.__stdout__  # restore stdout

        if self.time_old == None:
            self.trans_old = trans
            self.rot_old = rot
            self.time_old = t
            return None

        path_increment = sum([(axis - axis_old)**2 for axis, axis_old in zip(trans, self.trans_old)])**0.5
        time_increment = t - self.time_old

        if time_increment < rospy.Duration(0.5): # TODO make this a parameter
            return None

        velocity = path_increment / time_increment.to_sec()

        if self.velocity_old == None:
            self.trans_old = trans
            self.rot_old = rot
            self.time_old = t
            self.velocity_old = velocity
            return None

        acceleration = (velocity - self.velocity_old) / time_increment.to_sec()

        if self.acceleration_old == None:
            self.trans_old = trans
            self.rot_old = rot
            self.time_old = t
            self.velocity_old = velocity
            self.acceleration_old = acceleration
            return None

        jerk = (acceleration - self.acceleration_old) / time_increment.to_sec()

        # save old values for next step
        self.trans_old = trans
        self.rot_old = rot
        self.time_old = t
        self.velocity_old = velocity
        self.acceleration_old = acceleration

        return jerk

    def get_topics(self):
        return self.topics

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = self.name
        metric_result.unit = self.unit
        metric_result.mode = self.mode
        metric_result.status = self.status.status
        metric_result.series = []
        metric_result.groundtruth.available = self.groundtruth.available
        metric_result.groundtruth.data = self.groundtruth.data
        metric_result.groundtruth.epsilon = self.groundtruth.epsilon

        if self.status.status != TestblockStatus.SUCCEEDED:
            metric_result.groundtruth.result = Groundtruth.FAILED
            metric_result.groundtruth.error_message = metrics_helper.extract_error_message(self.status)
            return metric_result

        # check if result is available
        if len(self.series) == 0:
            # let the analyzer know that this test failed
            metric_result.groundtruth.result = Groundtruth.FAILED
            metric_result.groundtruth.error_message = "testblock {} stopped without result. " \
                                                      "No transforms found between {} & {}".format(self.testblock_name,
                                                                                                   self.root_frame,
                                                                                                   self.measured_frame)
            return metric_result

        # at this point we're sure that any result is available

        # set series
        if self.series_mode != None:
            metric_result.series = self.series

        # calculate metric data
        [metric_result.data, metric_result.min, metric_result.max, metric_result.mean, metric_result.std] = metrics_helper.calculate_metric_data(metric_result.name, metric_result.mode, self.series)

        # fill details as KeyValue messages
        details = []
        details.append(KeyValue("root_frame", self.root_frame))
        details.append(KeyValue("measured_frame", self.measured_frame))
        metric_result.details = details

        # evaluate metric data
        if metric_result.groundtruth.available: # groundtruth available
            if math.fabs(metric_result.groundtruth.data - metric_result.data.data) <= metric_result.groundtruth.epsilon:
                metric_result.groundtruth.result = Groundtruth.SUCCEEDED
                metric_result.groundtruth.error_message = "all OK"
            else:
                metric_result.groundtruth.result = Groundtruth.FAILED
                metric_result.groundtruth.error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data.data, metric_result.groundtruth.data, metric_result.groundtruth.epsilon)

        else: # groundtruth not available
            metric_result.groundtruth.result = Groundtruth.SUCCEEDED
            metric_result.groundtruth.error_message = "all OK (no groundtruth available)"

        return metric_result
