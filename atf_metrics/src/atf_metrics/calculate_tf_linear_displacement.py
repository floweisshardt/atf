#!/usr/bin/env python
import math
import os
import rospy
import sys
import tf
import tf2_msgs
import tf2_py
import tf2_ros

from tf import transformations

from atf_msgs.msg import MetricResult, KeyValue


class CalculateTfLinearDisplacementParamHandler(object):
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
                #rospy.logwarn_throttle(10, "No groundtruth parameters given, skipping groundtruth evaluation for metric 'path_length' in testblock '%s'"%testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculateTfLinearDisplacement(metric["topics"], metric["root_frame"], metric["measured_frame"], groundtruth, groundtruth_epsilon))
        return metrics



class CalculateTfLinearDisplacement(object):
    def __init__(self, topics, root_frame, measured_frame, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the distance covered by the given frame in relation to a given root frame.
        The tf data is sent over the tf topics given in the test_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """
        self.started = False
        self.finished = False
        self.active = False
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.topics = topics
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.path_length = 0.0
        self.first_value = True
        self.trans_old = []
        self.rot_old = []

        self.first_transform = None
        self.last_transform = None

        self.t = tf.Transformer(True, rospy.Duration(10.0))


    def start(self, status):
        self.active = True
        self.started = True
        self.first_transform = None


    def stop(self, status):
        self.active = False
        self.finished = True

        try:
            (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
            self.last_transform = (trans, rot)
        except Exception:
            pass


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
                #print "transform.header.stamp =", transform.header.stamp, "t =", t,
                #print "now =", rospy.Time.now(), "frame_id =", transform.header.frame_id,
                #print "child_frame_id =", transform.child_frame_id

        if self.active and self.first_transform is None:
            try:
                (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
                self.first_transform = (trans, rot)
            except Exception:
                pass


    def get_topics(self):
        return self.topics


    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = "tf_linear_displacement"
        metric_result.started = self.started  # FIXME remove
        metric_result.finished = self.finished  # FIXME remove
        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            lin_first = self.first_transform[0]
            lin_last = self.last_transform[0]

            metric_result.data = round(sum([(fl - ll)**2 for fl, ll in zip(lin_first, lin_last)])**0.5, 9)

            # fill details as KeyValue messages
            details = []
            details.append(KeyValue("root_frame", self.root_frame))
            details.append(KeyValue("measured_frame", self.measured_frame))
            details.append(KeyValue("lin_first", lin_first))
            details.append(KeyValue("lin_last", lin_last))
            metric_result.details = details

            # evaluate metric data
            if metric_result.data != None and metric_result.groundtruth != None and metric_result.groundtruth_epsilon != None:
                if math.fabs(metric_result.groundtruth - metric_result.data) <= metric_result.groundtruth_epsilon:
                    metric_result.groundtruth_result = True
                    metric_result.groundtruth_error_message = "all OK"
                else:
                    metric_result.groundtruth_result = False
                    metric_result.groundtruth_error_message = "groundtruth missmatch: %f not within %f+-%f"%(metric_result.data, metric_result.groundtruth, metric_result.groundtruth_epsilon)
                    #print metric_result.groundtruth_error_message

        if metric_result.data == None:
            metric_result.groundtruth_result = False
            metric_result.groundtruth_error_message = "no result"

        #print "\nmetric_result:\n", metric_result
        return metric_result
