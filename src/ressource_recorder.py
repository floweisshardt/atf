#!/usr/bin/python
import rospy
import psutil
import time
from cob_benchmarking.msg import RecordingManagerData


class RessourceRecorder:
    def __init__(self):

        self.pub_recording_manager_data = rospy.Publisher("recording_manager/ressources",
                                                          RecordingManagerData, queue_size=1)

        self.node_names = rospy.get_param(str(rospy.get_name()) + "/node_names")

        rospy.Timer(rospy.Duration.from_sec(0.1), self.collect_ressource_data)

    def collect_ressource_data(self, event):
        msg = RecordingManagerData()
        msg.id = "ressource_data"
        msg.timestamp = rospy.Time.from_sec(time.time())
        msg_data = ""

        pids = []
        try:
            for name in self.node_names:
                pids.append(self.get_pid(name))
        except IndexError:
            pass
        else:
            try:
                for pid in pids:
                    msg_data += str(psutil.Process(pid).name().split(".")[0]) + ";"
                    msg_data += str(psutil.Process(pid).cpu_percent(interval=0.01)) + ";"
                    msg_data += str(psutil.Process(pid).memory_percent()) + ";"
                    msg_data += str(psutil.Process(pid).io_counters()) + ";"
                    msg_data += str(psutil.net_io_counters()) + "|"
                msg.data = msg_data
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
            else:
                self.pub_recording_manager_data.publish(msg)

    @staticmethod
    def get_pid(name):
        pid = [p.pid for p in psutil.process_iter() if name in str(p.name)]
        return pid[0]

if __name__ == '__main__':
    rospy.init_node('ressource_recorder')
    rospy.loginfo("Starting 'Ressource recorder'...")
    RessourceRecorder()
    while not rospy.is_shutdown():
        rospy.spin()
