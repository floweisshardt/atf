#!/usr/bin/env python
import rospy

from copy import copy
#from obstacle_distance.srv import GetObstacleDistance


class RecordObstacleDistance:
    def __init__(self, topic_prefix, config_file, robot_config_file, write_lock, bag_file):
        self.topic_prefix = topic_prefix
        self.test_config = config_file
        self.robot_config_file = robot_config_file

        resources_timer_frequency = 100.0  # Hz
        self.timer_interval = 1/resources_timer_frequency
        self.res_pipeline = {}

        self.BfW = BagfileWriter(bag_file, write_lock)

        rospy.loginfo("Waiting for obstacle_distance node...")
        rospy.wait_for_service(self.robot_config_file["obstacle_distance"]["services"])
        #self.obstacle_distance_server = rospy.ServiceProxy(self.robot_config_file["obstacle_distance"]["services"],
                                                           GetObstacleDistance)

        rospy.Timer(rospy.Duration.from_sec(self.timer_interval), self.collect_obstacle_distances)

    def trigger_callback(self, msg):
        # Process Trigger
        pass

    def collect_obstacle_distances(self, event):
        pipeline = copy(self.res_pipeline)
        if not len(pipeline) == 0:


            #self.BfW.write_to_bagfile(topic, msg, rospy.Time.from_sec(time.time()))
            pass
