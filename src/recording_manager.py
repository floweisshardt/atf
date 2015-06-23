#!/usr/bin/python
import rospy
import rospkg
import yaml
from std_msgs.msg import Bool, String
import time


class RecordingManager:
    def __init__(self):
        sub_grasping_app = rospy.Subscriber("Recording_Manager/planning_timer", Bool, self.planning_timer)
        rospy.Subscriber("Recording_Manager/execution_timer", Bool, self.execution_timer)
        rospy.Subscriber("Recording_Manager/scene_informations", String, self.get_scene_infos)
        rospy.Subscriber("Recording_Manager/planning_error", Bool, self.error)

        self.path = rospkg.RosPack().get_path("cob_benchmarking") + "/results/scenes.yaml"

        self.time_start_planning = time
        self.time_planning = 0
        self.time_start_execution = time
        self.time_execution = 0
        self.scene_informations = {}
        self.scene_id = 0
        self.error_b = False

        while sub_grasping_app.get_num_connections() == 0:
            rospy.spin()

    def planning_timer(self, data):
        if data.data:
            self.time_start_planning = time.time()
        elif not data.data:
            self.time_planning = time.time() - self.time_start_planning
            print "Planning time: " + str(self.time_planning)

    def execution_timer(self, data):
        if data.data:
            self.time_start_execution = time.time()
        elif not data.data:
            self.time_execution = time.time() - self.time_start_execution
            print "Execution time: " + str(self.time_execution)

            self.write_results()

    def get_scene_infos(self, data):
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

if __name__ == '__main__':
    rospy.init_node('recording_manager')
    rospy.loginfo("Starting 'Recording manager'")
    RecordingManager()
    while not rospy.is_shutdown():
        rospy.spin()
