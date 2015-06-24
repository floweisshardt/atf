#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String
import psutil


class RessourceRecorder:
    def __init__(self):
        sub_check_manipulation_app = rospy.Subscriber("Recording_Manager/planning_timer", Bool)
        self.pub_ressource_data = rospy.Publisher("Recording_Manager/ressource_data", String, queue_size=1)
        rospy.Timer(rospy.Duration.from_sec(0.1), self.collect_ressource_data)

        node_names = ["grasping_test", "move_group"]

        # while sub_check_manipulation_app.get_num_connections() == 0:
            # rospy.spin()

        self.pids = []
        for name in node_names:
            self.pids.append(self.get_pid(name))
        '''
        while not rospy.is_shutdown():
            for pid in self.pids:
                print "Process '" + str(psutil.Process(pid).name()) + "': " + str(psutil.Process(pid).connections())
        '''

    def collect_ressource_data(self, event):
        msg = ""
        for pid in self.pids:
            msg += str(psutil.Process(pid).name().split(".")[0]) + ";"
            msg += str(psutil.Process(pid).cpu_percent(interval=0.1)) + ";"
            msg += str(psutil.Process(pid).memory_percent()) + ";"
            msg += str(psutil.Process(pid).io_counters()) + ";"
            msg += str(psutil.net_io_counters()) + "|"

        self.pub_ressource_data.publish(msg)

    @staticmethod
    def get_pid(name):
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]

if __name__ == '__main__':
    rospy.init_node('ressource_recorder')
    rospy.loginfo("Starting 'Ressource recorder'")
    RessourceRecorder()
    while not rospy.is_shutdown():
        rospy.spin()
