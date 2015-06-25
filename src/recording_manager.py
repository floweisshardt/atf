#!/usr/bin/python
import rospy
import rospkg
import yaml
from std_msgs.msg import Bool, String
import time
import rosbag
from threading import Lock
from tf2_msgs.msg import TFMessage

class RecordingManager:
    def __init__(self):

        self.time_start_planning = time
        self.time_planning = 0
        self.time_start_execution = time
        self.time_execution = 0
        self.error_b = False
        self.start_recording_tf = False
        self.start_ressources = False
        self.lock = Lock()

        self.scene = 1
        self.path_rb = rospkg.RosPack().get_path("cob_benchmarking") + "/results/scene_1.bag"
        self.bag = rosbag.Bag("init", 'w')
        self.bag.close()
        self.open_rosbag_file(self.path_rb)

        '''
        bag = rosbag.Bag(self.path_rb)
        self.planning = [msg for (topic, msg, t) in bag.read_messages(topics=['Recording_Manager/planning_timer'])]
        self.execution = [msg for (topic, msg, t) in bag.read_messages(topics=['Recording_Manager/execution_timer'])]
        self.scene_infos = [msg for (topic, msg, t)
                            in bag.read_messages(topics=['Recording_Manager/scene_informations'])]
        self.ressource_info = [msg for (topic, msg, t)
                               in bag.read_messages(topics=['Recording_Manager/ressource_data'])]
        self.tf_data = [msg for (topic, msg, t) in bag.read_messages(topics=['tf'])]
        bag.close()
        '''
        sub_grasping_app = rospy.Subscriber("Recording_Manager/planning_timer", Bool, callback=self.planning_timer,
                                            queue_size=1)
        rospy.Subscriber("Recording_Manager/execution_timer", Bool, self.execution_timer, queue_size=1)
        rospy.Subscriber("Recording_Manager/scene_informations", String, self.get_scene_infos, queue_size=1)
        rospy.Subscriber("Recording_Manager/planning_error", Bool, self.error, queue_size=1)
        rospy.Subscriber("Recording_Manager/ressource_data", String, self.current_ressources, queue_size=1)
        rospy.Subscriber("tf", TFMessage, self.current_tf_infos, queue_size=10)

        while sub_grasping_app.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.spin()

    def planning_timer(self, data):
        self.lock.acquire()
        self.bag.write("Recording_Manager/planning_timer", data)
        # self.write_to_rosbag(self.path_rb, "Recording_Manager/planning_timer", data)
        self.lock.release()

        if data.data:
            self.time_start_planning = time.time()
            self.start_ressources = True
            print "Planning started"
        elif not data.data:
            self.time_planning = time.time() - self.time_start_planning
            print "Planning stopped. Time: " + str(self.time_planning)

    def execution_timer(self, data):
        self.lock.acquire()
        self.bag.write("Recording_Manager/execution_timer", data)
        # self.write_to_rosbag(self.path_rb, "Recording_Manager/execution_timer", data)
        self.lock.release()

        if data.data:
            self.time_start_execution = time.time()
            self.start_recording_tf = True
            print "Execution started"
        elif not data.data:
            self.time_execution = time.time() - self.time_start_execution
            print "Execution stopped. Time: " + str(self.time_execution)

            self.start_recording_tf = False
            self.start_ressources = False

            self.bag.close()
            self.scene += 1
            self.path_rb = rospkg.RosPack().get_path("cob_benchmarking") + "/results/scene_" + str(self.scene) + ".bag"
            self.open_rosbag_file(self.path_rb)

    def current_tf_infos(self, data):
        if self.start_recording_tf:
            self.lock.acquire()
            self.bag.write("tf", data)
            # self.write_to_rosbag(self.path_rb, "tf", data)
            self.lock.release()

    def current_ressources(self, data):
        if self.start_ressources:
            self.lock.acquire()
            self.bag.write("Recording_Manager/ressource_data", data)
            # self.write_to_rosbag(self.path_rb, "Recording_Manager/ressource_data", data)
            self.lock.release()

    def get_scene_infos(self, data):
        self.lock.acquire()
        self.bag.write("Recording_Manager/scene_informations", data)
        # self.write_to_rosbag(self.path_rb, "Recording_Manager/scene_informations", data)
        self.lock.release()

    def error(self, data):
        if data.data:
            self.error_b = True

    def open_rosbag_file(self, path):
        try:
            self.bag = rosbag.Bag(path, "a")
        except rosbag.ROSBagException:
            self.bag = rosbag.Bag(path, "w")

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

    @staticmethod
    def save_data(filename, data):
        try:
            stream = file(filename, 'a')
        except IOError:
            yaml.dump(data, filename)
        else:
            yaml.dump(data, stream)

    @staticmethod
    def write_to_rosbag(path, topic, data):
        try:
            bag = rosbag.Bag(path, "a")
        except rosbag.ROSBagException:
            bag = rosbag.Bag(path, "w")

        try:
            bag.write(topic, data)
        finally:
            bag.close()

if __name__ == '__main__':
    rospy.init_node('recording_manager')
    rospy.loginfo("Starting 'Recording manager'")
    RecordingManager()
    while not rospy.is_shutdown():
        rospy.spin()
