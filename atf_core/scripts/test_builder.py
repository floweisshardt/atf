#!/usr/bin/env python
import rospy
import yaml
import rosparam
import os
import rosgraph
import rospkg

from atf_core import ATF, Testblock
import atf_metrics


def create_test_list():
    try:
        if not os.path.exists(rosparam.get_param("/analysing/result_yaml_output")):
            os.makedirs(rosparam.get_param("/analysing/result_yaml_output"))
    except rosgraph.masterapi.MasterError:
        pass

    if not os.path.exists(rosparam.get_param("/analysing/result_json_output")):
        os.makedirs(rosparam.get_param("/analysing/result_json_output"))

    test_config_path = rosparam.get_param("/analysing/test_config_file")
    config_data = load_data(test_config_path)[rosparam.get_param("/analysing/test_config")]
    metrics_data = load_data(rospkg.RosPack().get_path("atf_metrics") + "/config/metrics.yaml")

    get_test_list = []

    for testblock in config_data:
        metrics = []

        for metric in config_data[testblock]:
            try:
                metrics_data[metric]["argument"]
            except KeyError:
                metrics.append(getattr(atf_metrics, metrics_data[metric]["method"])())
            else:
                if metrics_data[metric]["method_per_argument"]:
                    for item in config_data[testblock][metric]:
                        metrics.append(getattr(atf_metrics, metrics_data[metric]["method"])(item))
                elif not metrics_data[metric]["method_per_argument"]:
                    metrics.append(getattr(atf_metrics, metrics_data[metric]["method"])(config_data[testblock][metric]))

        get_test_list.append(Testblock(testblock, metrics))

    return get_test_list


def load_data(filename):

        with open(filename, 'r') as stream:
            doc = yaml.load(stream)
        return doc


if __name__ == '__main__':
    rospy.init_node("test_framework")
    
    # Define metrics
    '''
    L1 = CalculatePathLength(["base_link", "gripper_right_grasp_link"])
    L2 = CalculatePathLength(["base_link", "arm_right_3_link"])
    L3 = CalculatePathLength(["base_link", "gripper_right_grasp_link"])
    L4 = CalculatePathLength(["arm_right_1_link", "gripper_right_grasp_link"])
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

    ATF(test_list).check_states()
