#!/usr/bin/env python

import rospy
import tf
import math
import yaml
import os.path

class robot_maker():
    def __init__(self):
        self.int_res = [0.35] #[0.25, 0.35, 0.5, 0.6, 0.75, 1.0, 1.5]
        self.part_num = [100] #[25, 50, 75, 100, 150, 200, 500]
        self.cb_counter = [1]
        self.mapping = [0.5, 1, 5, 10]
        self.ndt_match = ["true"]
        self.gain = [2]
        self.dyn_gain = [0.25, 0.5, 0.75, 1.0]
        self.file_path = "/home/fmw-hb/Desktop/robots/"  #'/home/fmw-hb/atf_catkin_ws/src/atf/hannes_test/config/robots/'
        self.robots = ""


    def make_robots(self):        
        for part in self.part_num:
            for res in self.int_res:
                for ndt in self.ndt_match:
                    for gain in self.gain:
                        for counter in self.cb_counter:
                            for dyn_gain in self.dyn_gain:
                                for mapping in self.mapping:

                                    if(ndt == "true"):
                                        filename = str(mapping)+"mapping_"+str(dyn_gain)+"overlaymap"
                                    else:
                                        filename = str(gain)+"gain_"+"part"+str(part)+"_"+str(int(res*1000))+"mm"
                                    # save everything in the file
                                    #with open(self.file_path+filename, 'w') as stream:
                                    #print "Path: "+self.file_path+filename
                                    print filename
                                    robot = open(self.file_path+filename+".yaml", 'w')
                                    robot.write("path_length:\n  topics:\n    - \"/tf\"\n    - \"/scan_unified\"\nrobot_bringup_launch: \"launch/all.launch\"\nwait_for_topics: []\nwait_for_services: []\n"
                                                 "additional_parameters:\n  \"/use_sim_time\": true\n"
                                                 "additional_arguments:\n  \"int_res\": "+str(res)+"\n  \"part_num\": "+str(part)+"\n  \"ndt_match\": "+ndt+"\n  \"gain\": "+str(gain)+
                                                "\n  \"counter\": "+str(counter)+"\n  \"mapping\": "+str(mapping)+"\n  \"stmgain\": "+str(dyn_gain))
                                        # stream.write(yaml.dump({'path_length': {'topics': [{'"/tf"', '"/scan_unified"'}]
                                        #                         'robot_bringup_launch': '"launch/all.launch"'
                                        #                         'wait_for_topics': '[]'
                                        #                         'wait_for_services': '[]'
                                        #                         'additional_parameters':
                                        #                           '"/use_sim_time"': true
                                        #                         additional_arguments:
                                        #                           "int_res": 0.35
                                        #                           "part_num": 75
                                        #                           "ndt_match": true}, default_flow_style=False))
                                    self.robots = self.robots + "    - " + filename + "\n"

        meta = open(self.file_path+"robots", 'wa')
        meta.write(self.robots)


if __name__ == '__main__':
    try:
        RM = robot_maker()
        RM.make_robots()
    except rospy.ROSInterruptException:
        pass
