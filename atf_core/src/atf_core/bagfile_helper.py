#!/usr/bin/env python
class BagfileWriter:
    def __init__(self,test_config, bagfile, write_lock):
        """
        Class for writing data to a rosbag file.
        :param bagfile: The bagfile to which the data should be written.
        :param write_lock: Lock for synchronised writing (from threading).
        :return:
        """
        self.lock = write_lock
        self.bag_file = bagfile

    def write_to_bagfile(self, topic, data, timestamp):
        """
        Function for writing data to a rosbag file.
        :param topic: The name of the topic.
        :type topic: str
        :param data: The data which should be written.
        :type data: Any
        :param timestamp: The time to which the data should be written into the rosbag file.
        :type timestamp: rospy.Time
        :return:
        """
        #print "bfw:", topic, timestamp
        self.lock.acquire()
        self.bag_file.write(topic, data, timestamp)
        self.lock.release()
