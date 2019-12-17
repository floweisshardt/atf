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
from tf.transformations import quaternion_multiply, quaternion_conjugate


from atf_msgs.msg import MetricResult, KeyValue



class CalculateTfIntegratedRotationParamHandler:
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
            metrics.append(CalculateTfIntegratedRotation(metric["topics"], metric["root_frame"], metric["measured_frame"], groundtruth, groundtruth_epsilon))
        return metrics



class CalculateTfIntegratedRotation:
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
        self.integrated_rotation = 0.0
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
                #print "transform.header.stamp =", transform.header.stamp, "t =", t, "now =", rospy.Time.now(), "frame_id =", transform.header.frame_id, "child_frame_id =", transform.child_frame_id
                self.t.setTransform(transform)

        # get path increment if testblock is active
        if self.active:
            self.get_rotation_increment()
                    
    def get_rotation_increment(self):
        try:
            sys.stdout = open(os.devnull, 'w') # supress stdout
            (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
        except tf2_ros.LookupException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric 'path_length' %s %s"%(type(e), e)
            return
        except tf2_py.ExtrapolationException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric 'path_length' %s %s"%(type(e), e)
            return
        except tf2_py.ConnectivityException as e:
            sys.stdout = sys.__stdout__  # restore stdout
            #print "Exception in metric 'path_length' %s %s"%(type(e), e)
            return
        except Exception as e:
            sys.stdout = sys.__stdout__  # restore stdout
            print "general exeption in calculate_path_length:", type(e), e
            return
        sys.stdout = sys.__stdout__  # restore stdout

        if self.first_value:
            self.trans_old = trans
            self.rot_old = rot
            self.first_value = False
            return

        diff_q = quaternion_multiply(rot, quaternion_conjugate(self.rot_old))
        diff_e = transformations.euler_from_quaternion(diff_q)
        eucl_dist = sum([en**2 for en in diff_e])**0.5
        self.integrated_rotation += eucl_dist

        self.trans_old = trans
        self.rot_old = rot

    def get_topics(self):
        return self.topics

    def get_result(self):
        metric_result = MetricResult()
        metric_result.name = "tf_integrated_rotation"
        metric_result.started = self.started # FIXME remove
        metric_result.finished = self.finished # FIXME remove
        metric_result.data = None
        metric_result.groundtruth = self.groundtruth
        metric_result.groundtruth_epsilon = self.groundtruth_epsilon
        
        # assign default value
        metric_result.groundtruth_result = None
        metric_result.groundtruth_error_message = None

        if metric_result.started and metric_result.finished: #  we check if the testblock was ever started and stopped
            # calculate metric data
            metric_result.data = round(self.integrated_rotation, 4)

            # fill details as KeyValue messages
            details = []
            details.append(KeyValue("root_frame", self.root_frame))
            details.append(KeyValue("measured_frame", self.measured_frame))
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
