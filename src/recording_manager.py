#!/usr/bin/python
import rospy
import rospkg
from cob_benchmarking.msg import RecordingManagerData
import datetime
import rosbag
from threading import Lock
from tf2_msgs.msg import TFMessage


class RecordingManager:
    def __init__(self):

        self.record_tf = False
        self.record_ressources = False
        self.lock = Lock()

        self.scene = "scene " + str(datetime.datetime.now()) + ".bag"
        self.scene = self.scene.replace(" ", "_")
        self.path_rb = rospkg.RosPack().get_path("cob_benchmarking") + "/results/"
        self.path_rb += self.scene
        self.bag = rosbag.Bag(self.path_rb, 'w')

        '''
        bag = rosbag.Bag(self.path_rb)
        self.planning = [msg for (topic, msg, t) in bag.read_messages(topics=['planning_timer'])]
        self.execution = [msg for (topic, msg, t) in bag.read_messages(topics=['execution_timer'])]
        self.scene_infos = [msg for (topic, msg, t)
                            in bag.read_messages(topics=['scene_informations'])]
        self.ressource_info = [msg for (topic, msg, t)
                               in bag.read_messages(topics=['ressource_data'])]
        self.tf_data = [msg for (topic, msg, t) in bag.read_messages(topics=['tf'])]
        bag.close()
        '''
        rospy.Subscriber("recording_manager_data", RecordingManagerData, self.data_callback, queue_size=10)
        rospy.Subscriber("tf", TFMessage, self.current_tf_infos, queue_size=10)

    def data_callback(self, msg):

        if msg.id.data != "ressource_data":
            self.lock.acquire()
            self.bag.write(msg.id.data, msg)
            self.lock.release()
        elif self.record_ressources:
            self.lock.acquire()
            self.bag.write(msg.id.data, msg)
            self.lock.release()

        if msg.id.data == "planning_timer" and msg.status.data:
            self.record_ressources = True
            rospy.loginfo("Planning started")

        elif msg.id.data == "planning_timer" and not msg.status.data:
            rospy.loginfo("Planning stopped")

        if msg.id.data == "execution_timer" and msg.status.data:
            self.record_tf = True
            rospy.loginfo("Execution started")

        elif (msg.id.data == "execution_timer" and not msg.status.data) or msg.id.data == "planning_error":
            self.record_ressources = False
            self.record_tf = False
            rospy.loginfo("Execution stopped")

            self.bag.close()

            self.scene = "scene " + str(datetime.datetime.now()) + ".bag"
            self.scene = self.scene.replace(" ", "_")
            self.path_rb = rospkg.RosPack().get_path("cob_benchmarking") + "/results/"
            self.path_rb += self.scene
            self.bag = rosbag.Bag(self.path_rb, 'w')

    def current_tf_infos(self, data):
        if self.record_tf:
            self.lock.acquire()
            self.bag.write("tf", data)
            self.lock.release()

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
    rospy.init_node('recording_manager')
    rospy.loginfo("Starting 'Recording manager'...")
    RecordingManager()
    while not rospy.is_shutdown():
        rospy.spin()
