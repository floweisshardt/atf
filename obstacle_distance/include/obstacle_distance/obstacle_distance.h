#ifndef OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
#define OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H

class ObstacleDistance
{
public:
    ObstacleDistance();
    ~ObstacleDistance();

    void getDistanceToObstacles(std::vector<std::string> link_names);

private:
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    std::string PLANNING_SCENE_SERVICE;
};

#endif //OBSTACLE_DISTANCE_OBSTACLE_DISTANCE_H
