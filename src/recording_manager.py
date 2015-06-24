#!/usr/bin/python
import rospy
import rospkg
import yaml
from std_msgs.msg import Bool, String, Int32
import time
import rosbag
from threading import Lock
from sensor_msgs.msg import JointState

class RecordingManager:
    def __init__(self):

        self.path = rospkg.RosPack().get_path("cob_benchmarking") + "/results/scenes.yaml"

        self.time_start_planning = time
        self.time_planning = 0
        self.time_start_execution = time
        self.time_execution = 0
        self.scene_informations = {}
        self.scene_id = 0
        self.error_b = False
        self.path_rosbag = rospkg.RosPack().get_path("cob_benchmarking") + "/results/scene.bag"
        self.start_recording_joints = False
        self.start_ressources = False
        self.lock = Lock()
        '''
        bag = rosbag.Bag(self.path_rosbag)
        for topic, msg, t in bag.read_messages(topics=['Recording_Manager/planning_timer',
                                                       'Recording_Manager/execution_timer',
                                                       'Recording_Manager/scene_informations',
                                                       'Recording_Manager/ressource_data']):
            print msg.data
        bag.close()
        bag = rosbag.Bag(self.path_rosbag)

        for topic, msg, t in bag.read_messages(topics=['arm_right/joint_states',
                                                       'arm_left/joint_states']):
            print msg
        bag.close()
        '''

        sub_grasping_app = rospy.Subscriber("Recording_Manager/planning_timer", Bool, callback=self.planning_timer,
                                            queue_size=10)
        rospy.Subscriber("Recording_Manager/execution_timer", Bool, self.execution_timer, queue_size=10)
        rospy.Subscriber("Recording_Manager/scene_informations", String, self.get_scene_infos, queue_size=10)
        rospy.Subscriber("Recording_Manager/planning_error", Bool, self.error, queue_size=10)
        rospy.Subscriber("arm_left/joint_states", JointState, self.current_joints_left, queue_size=10)
        rospy.Subscriber("arm_right/joint_states", JointState, self.current_joints_right, queue_size=10)
        rospy.Subscriber("Recording_Manager/ressource_data", String, self.current_ressources, queue_size=10 )

        while sub_grasping_app.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.spin()

    def planning_timer(self, data):
        self.lock.acquire()
        self.write_to_rosbag(self.path_rosbag, "Recording_Manager/planning_timer", data)
        self.lock.release()

        if data.data:
            self.time_start_planning = time.time()
            self.start_ressources = True
        elif not data.data:
            self.time_planning = time.time() - self.time_start_planning
            print "Planning time: " + str(self.time_planning)

    def execution_timer(self, data):
        self.lock.acquire()
        self.write_to_rosbag(self.path_rosbag, "Recording_Manager/execution_timer", data)
        self.lock.release()

        if data.data:
            self.time_start_execution = time.time()
            self.start_recording_joints = True
        elif not data.data:
            self.time_execution = time.time() - self.time_start_execution
            print "Execution time: " + str(self.time_execution)

            self.start_recording_joints = False
            self.start_ressources = False
            self.write_results()

    def current_joints_left(self, data):
        if self.start_recording_joints:
            self.lock.acquire()
            self.write_to_rosbag(self.path_rosbag, "arm_left/joint_states", data)
            self.lock.release()

    def current_joints_right(self, data):
        if self.start_recording_joints:
            self.lock.acquire()
            self.write_to_rosbag(self.path_rosbag, "arm_right/joint_states", data)
            self.lock.release()

    def current_ressources(self, data):
        if self.start_ressources:
            self.lock.acquire()
            self.write_to_rosbag(self.path_rosbag, "Recording_Manager/ressource_data", data)
            self.lock.release()

    def get_scene_infos(self, data):
        self.lock.acquire()
        self.write_to_rosbag(self.path_rosbag, "Recording_Manager/scene_informations", data)
        self.lock.release()

        self.scene_informations = self.create_dict(data.data)

    def write_results(self):
        self.scene_id += 1
        name = "test_" + str(self.scene_id)
        data = {name: {"scene": self.scene_informations["scene"],
                       "additional_obstacles": self.scene_informations["additional_obstacles"],
                       "arm": self.scene_informations["arm"],
                       "planning_method": self.scene_informations["planning_method"],
                       "planer_id": self.scene_informations["planer_id"],
                       "eef_step": self.scene_informations["eef_step"],
                       "jump_threshold": self.scene_informations["jump_threshold"],
                       "planning_time": self.time_planning,
                       "execution_time": self.time_execution,
                       "planning_failed": self.error_b}}

        self.save_data(self.path, data)
        self.error_b = False
        self.scene_informations = {}

    def error(self, data):
        if data.data:
            self.error_b = True
            self.write_results()

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
