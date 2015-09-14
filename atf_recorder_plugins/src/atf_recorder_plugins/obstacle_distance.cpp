#include <atf_recorder_plugins/obstacle_distance.h>

class CreateCollisionWorld: public collision_detection::CollisionWorldFCL
{
public:
    CreateCollisionWorld(const collision_detection::WorldPtr & world) :
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

class CreateCollisionRobot: public collision_detection::CollisionRobotFCL
{
public:
    CreateCollisionRobot(const robot_model::RobotModelConstPtr &model) :
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

void ObstacleDistance::getDistanceToObstacles(const ros::TimerEvent&)
{
    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());

    for (int i = 0; i < current_joint_states.name.size(); i++)
    {
        robot_state.setVariablePosition(current_joint_states.name[i], current_joint_states.position[i]);
    }

    robot_state.update();
    robot_state.updateCollisionBodyTransforms();

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;

    collision_robot.getCollisionObject(robot_state, robot_obj);
    collision_world.getCollisionObject(world_obj);

    atf_msgs::ObstacleDistance ob;

    for (int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData* robot_link = static_cast<const collision_detection::CollisionGeometryData*>(robot_obj[i]->collisionGeometry()->getUserData());

        atf_msgs::ObstacleDistanceLink ob_link;
        ob_link.name = robot_link->getID();

        for (int j = 0; j < world_obj.size(); j++)
        {
            const collision_detection::CollisionGeometryData* collision_object = static_cast<const collision_detection::CollisionGeometryData*>(world_obj[j]->collisionGeometry()->getUserData());
            fcl::DistanceResult res;
            res.update(2.0, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

            double dist = fcl::distance(robot_obj[i].get(), world_obj[j].get(), fcl::DistanceRequest(), res);

            ob_link.objects.push_back(collision_object->getID());
            ob_link.distances.push_back(dist);
        }
        ob.links.push_back(ob_link);
    }
    obstacle_distance_publisher.publish(ob);
}

void ObstacleDistance::joint_state_callback(const sensor_msgs::JointStatePtr &joint_states)
{
    current_joint_states = *joint_states;
}

void ObstacleDistance::getPlanningScene()
{
    while(ros::ok())
    {
        planning_scene_monitor_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    }
}

ObstacleDistance::ObstacleDistance()
        : ros::NodeHandle()
{
    PLANNING_SCENE_SERVICE = "get_planning_scene";
    double publish_frequency = 100.0; //Hz

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_ =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_listener_);

    //Initialize timer & subscriber
    joint_state_subscriber = subscribe("joint_states", 1, &ObstacleDistance::joint_state_callback, this);
    obstacle_distance_publisher = advertise<atf_msgs::ObstacleDistance>("/atf/obstacle_distance", 1);
    obstacle_distance_timer = createTimer(ros::Duration(1/publish_frequency), &ObstacleDistance::getDistanceToObstacles, this);

    //Initialize thread for updating the planning scene
    boost::thread* ps_thread = new boost::thread(&ObstacleDistance::getPlanningScene, this);
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "obstacle_distance");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
