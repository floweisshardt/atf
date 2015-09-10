#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <obstacle_distance/obstacle_distance.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

//Class inherited from collision world fcl
class MyCollisionWorld: public collision_detection::CollisionWorldFCL
{
public:
    MyCollisionWorld(const collision_detection::WorldPtr & world) :
            CollisionWorldFCL(world)
    {
    }
    void getCollisionObject(std::vector<boost::shared_ptr<fcl::CollisionObject> > & obj)
    {
        std::map<std::string, collision_detection::FCLObject>::iterator it = fcl_objs_.begin();
        obj.reserve(fcl_objs_.size());

        for (it; it != fcl_objs_.end(); ++it)
        {
            obj.push_back(it->second.collision_objects_[0]);
        }
    }
};

//Class inherited from collision robot fcl
class MyCollisionRobot: public collision_detection::CollisionRobotFCL
{
public:
    MyCollisionRobot(const robot_model::RobotModelConstPtr &model) :
            CollisionRobotFCL(model)
    {
    }
    void getCollisionObject(const robot_state::RobotState & state,
                            std::vector<boost::shared_ptr<fcl::CollisionObject> > & obj)
    {
        collision_detection::FCLObject fcl_obj;
        constructFCLObject(state, fcl_obj);
        obj = fcl_obj.collision_objects_;
    }
};

void ObstacleDistance::getDistanceToObstacles(std::vector<std::string> link_names)
{
    planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    ROS_INFO("Service requested");
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();

    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    MyCollisionWorld cworld(planning_scene_ptr->getWorldNonConst());
    robot_state::RobotState state(planning_scene_ptr->getCurrentState());
    // setJointPositions(const std::string &joint_name, const double *position)
    state.update();
    state.updateCollisionBodyTransforms();

    MyCollisionRobot crobot(state.getRobotModel());
    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;

    crobot.getCollisionObject(state, robot_obj);
    cworld.getCollisionObject(world_obj);

    fcl::DistanceRequest req;
    fcl::DistanceResult res;
    double d;

    for (int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData* cd = static_cast<const collision_detection::CollisionGeometryData*>(robot_obj[i]->collisionGeometry()->getUserData());

        for (int it = 0; it < link_names.size(); it++)
        {
            if (link_names[it].compare(cd->getID()) == 0)
            {
                for (int j = 0; j < world_obj.size(); j++)
                {
                    double dist = fcl::distance(robot_obj[i].get(), world_obj[j].get(), req, res);

                    //std::cout << "Robot Link " << cd->getID() << " at (" << robot_obj[i]->getTranslation().data.vs[0] << ", " << robot_obj[i]->getTranslation().data.vs[1] << ", " << robot_obj[i]->getTranslation().data.vs[2] << ")" << std::endl;
                    //std::cout << "Object at (" << world_obj[j]->getTranslation().data.vs[0] << ", " << world_obj[j]->getTranslation().data.vs[1] << ", " << world_obj[j]->getTranslation().data.vs[2] << ")" << std::endl;
                    std::cout << cd->getID() << ": Minimal distance " << dist << std::endl;
                }
            }
        }
    }
    ROS_INFO("Calculation done");
}

ObstacleDistance::ObstacleDistance()
{
    PLANNING_SCENE_SERVICE = "get_planning_scene";

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_ =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description"/*, tf_listener_*/);
    planning_scene_monitor_->startSceneMonitor("planning_scene");

};

ObstacleDistance::~ObstacleDistance()
{}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "obstacle_distance");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ObstacleDistance *ob = new ObstacleDistance();

    std::string link_names[] = {"gripper_right_grasp_link", "gripper_left_grasp_link"};
    std::vector<std::string> links (link_names, link_names + sizeof(link_names) / sizeof(std::string));

    while(ros::ok())
    {
        ob->getDistanceToObstacles(links);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    ros::shutdown();
    return 0;
}
