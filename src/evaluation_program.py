#!/usr/bin/python
import rospy
import rospkg
import rosbag
from cob_benchmarking.msg import RecordingManagerData
from tf2_msgs.msg import TFMessage


class EvaluationProgram:
    def __init__(self):

        self.path_rb = ""

    def read_bagfile(self):

        bag = rosbag.Bag(self.path_rb)
        planning = [msg for (topic, msg, t) in bag.read_messages(topics=['planning_timer'])]
        execution = [msg for (topic, msg, t) in bag.read_messages(topics=['execution_timer'])]
        scene_infos = [msg for (topic, msg, t)
                       in bag.read_messages(topics=['scene_informations'])]
        ressource_info = [msg for (topic, msg, t)
                          in bag.read_messages(topics=['ressource_data'])]
        tf_data = [msg for (topic, msg, t) in bag.read_messages(topics=['tf'])]
        bag.close()

    @staticmethod
    def create_dict(msg):
        values = msg.split(";")
        dic = {}
        i = 0
        for item in values:
            i += 1
            if i == len(values):
                break
            temp = item.split(":")
            dic[temp[0]] = temp[1]
        return dic


if __name__ == '__main__':
    rospy.init_node('evaluation_program')
    EvaluationProgram()
    while not rospy.is_shutdown():
        rospy.spin()
