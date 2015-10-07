#ifndef ATF_RECORDER_PLUGINS_OBSTACLE_DISTANCE_H
#define ATF_RECORDER_PLUGINS_OBSTACLE_DISTANCE_H

#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

#include <boost/thread.hpp>

#include <sensor_msgs/JointState.h>
#include <atf_msgs/ObstacleDistance.h>
#include <atf_msgs/ObstacleDistanceLink.h>

class ObstacleDistance : public ros::NodeHandle {
public:
    ObstacleDistance();

private:
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::string PLANNING_SCENE_SERVICE;
    float MAXIMAL_MINIMAL_DISTANCE;

    ros::Subscriber joint_state_subscriber_;
    ros::Publisher obstacle_distance_publisher_;
    ros::Timer obstacle_distance_timer_;
    sensor_msgs::JointState current_joint_states_;

    void joint_state_callback(const sensor_msgs::JointStatePtr &joint_states);

    void getDistanceToObstacles(const ros::TimerEvent &);

    void getPlanningScene();

};

#endif //ATF_RECORDER_PLUGINS_OBSTACLE_DISTANCE_H
