#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import time

class RecordingManager:
    def __init__(self):
        rospy.Subscriber("Recording_Manager/planning_timer", Bool, self.planning_timer)
        rospy.Subscriber("Recording_Manager/execution_timer", Bool, self.execution_timer)

        # self.time_start_planning = rospy.Time()
        # self.time_start_execution = rospy.Time()
        self.time_start_planning = time
        self.time_start_execution = time

    def planning_timer(self, data):
        if data.data:
            # self.time_start_planning = rospy.Time.now()
            self.time_start_planning = time.time()
        elif not data.data:
            # print "Planning time ros: " + str((rospy.Time.now() - self.time_start_planning).to_sec())
            print "Planning time: " + str(time.time() - self.time_start_planning)

    def execution_timer(self, data):
        if data.data:
            # self.time_start_execution = rospy.Time.now()
            self.time_start_execution = time.time()
        elif not data.data:
            # print "Execution time ros: " + str((rospy.Time.now() - self.time_start_execution).to_sec())
            print "Execution time: " + str(time.time() - self.time_start_execution)

if __name__ == '__main__':
    rospy.init_node('recording_manager')
    rospy.loginfo("Starting 'Recording manager'")
    RecordingManager()
    while not rospy.is_shutdown():
        rospy.spin()