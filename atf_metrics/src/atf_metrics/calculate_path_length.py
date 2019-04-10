#!/usr/bin/env python
import rospy
import tf
import math
import tf2_msgs
import threading

class CalculatePathLengthParamHandler:
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
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'path_length' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            metrics.append(CalculatePathLength(metric["topic"], metric["root_frame"], metric["measured_frame"], groundtruth, groundtruth_epsilon))
        return metrics

class CalculatePathLength:
    def __init__(self, topic, root_frame, measured_frame, groundtruth, groundtruth_epsilon):
        """
        Class for calculating the distance covered by the given frame in relation to a given root frame.
        The tf data is sent over the tf topic given in the robot_config.yaml.
        :param root_frame: name of the first frame
        :type  root_frame: string
        :param measured_frame: name of the second frame. The distance will be measured in relation to the root_frame.
        :type  measured_frame: string
        """

        self.active = False
        self.topic = topic
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.path_length = 0.0
        self.tf_sampling_freq = 20.0  # Hz
        self.first_value = True
        self.trans_old = []
        self.rot_old = []
        self.groundtruth = groundtruth
        self.groundtruth_epsilon = groundtruth_epsilon
        self.finished = False

        #self.listener = tf.TransformListener()
        self.t = tf.Transformer(True, rospy.Duration(10.0))

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

    def update(self, topic, msg, t):
        #print "update"
        if self.active:
            #print "active"
            #print "topic=", topic, "self.topic=", self.topic
            if topic == self.topic:
                #print "topic match"
                # spawn two threads
                #thread_tf = threading.Thread(target=self.update_tf, args=(msg,))
                #thread_path_length = threading.Thread(target=self.update_path_length, args=(msg,))
                #thread_tf.start()
                #thread_path_length.start()
                self.update_path_length(msg)


    def update_tf(self, msg):
        #print "update tf"
        for transform in msg.transforms:
            #print "transform=", transform
            self.t.setTransform(transform)

    def update_path_length(self, msg):
        #print "update path_length"
        self.update_tf(msg)

        #self.t.waitForTransform(self.root_frame,
        #                        self.measured_frame,
        #                        rospy.Time(0),
        #                        rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))

        

        (trans, rot) = self.t.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

        if self.first_value:
            self.trans_old = trans
            self.rot_old = rot
            self.first_value = False
            return
        #print "transformations: \n", "trans[0]", trans[0], "self.trans_old[0]",self.trans_old[0], "trans[1]", trans[1], "self.trans_old[1]",self.trans_old[1], "trans[2]",trans[2], "self.trans_old[2]",self.trans_old[2], "\n ------------------------------------------------ "
        path_increment = math.sqrt((trans[0] - self.trans_old[0]) ** 2 + (trans[1] - self.trans_old[1]) ** 2 +
                                (trans[2] - self.trans_old[2]) ** 2)
        if(path_increment < 1):
            #rospy.logwarn("Transformation: %s, Path Increment: %s",str(trans), str(path_increment))
            self.path_length += path_increment

        else:
            rospy.logwarn("Transformation Failed! \n Transformation: %s, Path Increment: %s",str(trans), str(path_increment))

        self.trans_old = trans
        self.rot_old = rot

        #print self.path_length


                    #print self.t.getFrameStrings()
            
            # try:
            #     self.listener.waitForTransform(self.root_frame,
            #                                    self.measured_frame,
            #                                    rospy.Time(0),
            #                                    rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))
            #     (trans, rot) = self.listener.lookupTransform(self.root_frame, self.measured_frame, rospy.Time(0))

            # except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            #     rospy.logwarn(e)
            #     pass
            # else:
            #     if self.first_value:
            #         self.trans_old = trans
            #         self.rot_old = rot
            #         self.first_value = False
            #         return
            #     #print "transformations: \n", "trans[0]", trans[0], "self.trans_old[0]",self.trans_old[0], "trans[1]", trans[1], "self.trans_old[1]",self.trans_old[1], "trans[2]",trans[2], "self.trans_old[2]",self.trans_old[2], "\n ------------------------------------------------ "
            #     path_increment = math.sqrt((trans[0] - self.trans_old[0]) ** 2 + (trans[1] - self.trans_old[1]) ** 2 +
            #                                (trans[2] - self.trans_old[2]) ** 2)
            #     if(path_increment < 1):
            #         #rospy.logwarn("Transformation: %s, Path Increment: %s",str(trans), str(path_increment))
            #         self.path_length += path_increment

            #     else:
            #         rospy.logwarn("Transformation Failed! \n Transformation: %s, Path Increment: %s",str(trans), str(path_increment))

            #     self.trans_old = trans
            #     self.rot_old = rot

    def get_result(self):
        groundtruth_result = None
        details = {"root_frame": self.root_frame, "measured_frame": self.measured_frame}
        if self.finished:
            data = round(self.path_length, 3)
            if self.groundtruth != None and self.groundtruth_epsilon != None:
                if math.fabs(self.groundtruth - data) <= self.groundtruth_epsilon:
                    groundtruth_result = True
                else:
                    groundtruth_result = False
            return "path_length", data, groundtruth_result, self.groundtruth, self.groundtruth_epsilon, details
        else:
            return False

#class TFHandler(threading.Thread)
#    def __init__(self, number): 
#        threading.Thread.__init__(self) 
#        self.msg = None
# 
#    def run(self):
#        if
#        #for transform in msg.transforms:
#        #    #print "transform=", transform
#    #    self.t.setTransform(transform)
    