#!/usr/bin/env python
import math
import rospy
import tf


class CalculatePathLengthParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        self.params = []

    def parse_parameter(self, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        self.params = params
        metrics = []

        for item in self.params:
            metrics.append(CalculatePathLength(item[0], item[1]))

        return metrics


class CalculatePathLength:
    def __init__(self, root_frame, measured_frame):
        """
        Class for calculating the distance covered by the given frame in relation to a given root frame.
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """

        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.path_length = 0.0
        self.tf_sampling_freq = 100.0  # Hz
        self.first_value = True
        self.trans_old = []
        self.rot_old = []
        self.finished = False

        self.listener = tf.TransformListener()

        rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf)

    def start(self):
        self.active = True

    def stop(self):
        self.active = False
        self.finished = True

    def pause(self):
        self.active = False
        self.first_value = True

    @staticmethod
    def purge():
        pass

    def record_tf(self, event):
        if self.active:
            try:

                self.listener.waitForTransform(self.root_frame,
                                               self.measured_frame,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(2 / self.tf_sampling_freq))
                (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, Exception), e:
                rospy.logwarn(e)
            else:
                if self.first_value:
                    self.trans_old = trans
                    self.rot_old = rot
                    self.first_value = False
                    return

                path_increment = math.sqrt((trans[0] - self.trans_old[0]) ** 2 + (trans[1] - self.trans_old[1]) ** 2 +
                                           (trans[2] - self.trans_old[2]) ** 2)
                self.path_length += path_increment

                self.trans_old = trans
                self.rot_old = rot

    def get_result(self):
        if self.finished:
            return "path_length", {self.root_frame + " to " + self.measured_frame: round(self.path_length, 3)}
        else:
            return False
