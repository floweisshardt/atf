#ifndef OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
#define OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

#include <boost/thread.hpp>
#include <sensor_msgs/JointState.h>

class ObstacleDistance : public ros::NodeHandle
{
public:
    ObstacleDistance();
    ~ObstacleDistance();

private:
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::string PLANNING_SCENE_SERVICE;

    ros::Subscriber tf_subscriber;
    ros::Timer distance_timer;
    std::vector<std::string> requested_links;
    sensor_msgs::JointState current_joint_states;

    void joint_state_callback(const sensor_msgs::JointStatePtr &joint_states);
    void getDistanceToObstacles(const ros::TimerEvent&);
    void getPlanningScene();

};

#endif //OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
