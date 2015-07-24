#!/usr/bin/env python
import rospy
import rospkg
import yaml
import rosparam

from atf_testing import ATF, Testblock
from atf_metrics import CalculatePathLength, CalculateTime, CalculateResources


def create_test_list():

    test_config_path = rospkg.RosPack().get_path("atf_recorder") + "/config/test_config.yaml"
    config_data = load_data(test_config_path)[rosparam.get_param("/test_name")]

    test_list_int = []

    for testblock in config_data:
        metrics = []
        # Time
        try:
            config_data[testblock]["time"]
        except KeyError:
            pass
        else:
            metrics.append(CalculateTime())

        # Path length
        try:
            config_data[testblock]["path_length"]
        except KeyError:
            pass
        else:
            for item in config_data[testblock]["path_length"]:
                metrics.append(CalculatePathLength(item[0], item[1]))

        # Resources
        try:
            config_data[testblock]["resources"]
        except KeyError:
            pass
        else:
            resource = {}
            for item in config_data[testblock]["resources"]:
                resource.update({item: config_data[testblock]["resources"][item]})
            metrics.append(CalculateResources(resource))

        test_list_int.append(Testblock(testblock, metrics))

    return test_list_int


def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc


if __name__ == '__main__':
    rospy.init_node("test_framework")
    
    # Define metrics
    '''
    L1 = CalculatePathLength("base_link", "gripper_right_grasp_link")
    L2 = CalculatePathLength("base_link", "arm_right_3_link")
    L3 = CalculatePathLength("base_link", "gripper_right_grasp_link")
    L4 = CalculatePathLength("arm_right_1_link", "gripper_right_grasp_link")
    T1 = CalculateTime()
    T2 = CalculateTime()
    T3 = CalculateTime()
    T4 = CalculateTime()
    T5 = CalculateTime()
    T6 = CalculateTime()
    T7 = CalculateTime()
    T8 = CalculateTime()
    R1 = CalculateResources({"cpu":["move_group"], "mem":["move_group"], "io":["move_group"], "network":["move_group"]})

    D1 = CalculateDistanceToObstacles()
    '''

    test_list = create_test_list()

    '''
    test_list = [Testblock("execution_2", [L1, L2, T1, R1]),
                 Testblock("planning_all", [T2]),
                 Testblock("planning_1", [T3]),
                 Testblock("planning_2", [T4]),
                 Testblock("planning_3", [T5]),
                 Testblock("execution_all", [T6, L3, L4]),
                 Testblock("execution_1", [T7]),
                 Testblock("execution_3", [T8])]
    '''

    ATF(test_list).wait_for_end()
