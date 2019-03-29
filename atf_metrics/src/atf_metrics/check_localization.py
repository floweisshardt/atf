#!/usr/bin/env python
import rospy
import tf
import math

class CheckLocalizationParamHandler:
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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'distance' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CheckLocalization(metric["root_frame"], metric["measured_frame"], metric["max_loc_error"], groundtruth, groundtruth_epsilon))
        return metrics

class CheckLocalization:
    def __init__(self, root_frame, measured_frame, max_loc_error, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the seconds passed with lost localization in respect to a given root frame.
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        :param max_loc_error: distance between the two frames which is considered to be localization lost
        :type max_loc_error:float
        """

        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.distance = 0.0
        self.lost_count = 0
        self.tf_sampling_freq = 1.0 # Hz
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.max_loc_error = max_loc_error;
        self.finished = False

        self.listener = tf.TransformListener()

        rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf)

    def start(self, timestamp):
        self.active = True

    def stop(self, timestamp):
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        self.active = False
        self.first_value = True

    def purge(self, timestamp):
        pass

    def record_tf(self, event):
        if self.active:
            try:
                self.listener.waitForTransform(self.root_frame,
                                               self.measured_frame,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))
                (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                #rospy.logwarn(e)
                pass
            else:
                self.distance = math.sqrt(trans[0]**2 + trans[1]**2)
                if (self.distance > self.max_loc_error):
                    self.lost_count += 1


    def get_result(self):
        groundtruth_result = None
        details = {"root_frame": self.root_frame, "measured_frame": self.measured_frame}
        if self.finished:
            data = self.lost_count
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "localization", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False
