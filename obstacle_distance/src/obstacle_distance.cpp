#include <obstacle_distance/obstacle_distance.h>

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
    //planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();

    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());
    std::vector<std::string> requested_links;
    for (int i = 0; i < current_joint_states.name.size(); i++)
    {
        robot_state.setVariablePosition(current_joint_states.name[i], current_joint_states.position[i]);
        requested_links.push_back(current_joint_states.name[i].c_str());
    }

    robot_state.update();
    robot_state.updateCollisionBodyTransforms();

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;

    collision_robot.getCollisionObject(robot_state, robot_obj);
    collision_world.getCollisionObject(world_obj);

    fcl::DistanceRequest req;
    fcl::DistanceResult res;
    atf_msgs::ObstacleDistance ob;

    requested_links.push_back("arm_right_1_link");
    requested_links.push_back("arm_right_2_link");
    requested_links.push_back("arm_right_3_link");
    requested_links.push_back("arm_right_4_link");
    requested_links.push_back("arm_right_5_link");
    requested_links.push_back("arm_right_6_link");
    requested_links.push_back("arm_right_7_link");
    requested_links.push_back("gripper_right_grasp_link");

    for (int i = 0; i < robot_obj.size(); i++)
    {
        const collision_detection::CollisionGeometryData* cd = static_cast<const collision_detection::CollisionGeometryData*>(robot_obj[i]->collisionGeometry()->getUserData());

        for (int it = 0; it < requested_links.size(); it++)
        {
            if (requested_links[it].compare(cd->getID()) == 0)
            {
                for (int j = 0; j < world_obj.size(); j++)
                {
                    double dist = fcl::distance(robot_obj[i].get(), world_obj[j].get(), req, res);
                    ROS_INFO("Minimal distance for %s: %f", cd->getID().c_str(), dist);
                }

            }
        }

        /*
        atf_msgs::ObstacleDistanceLink ob_link;
        ob_link.name = robot_link->getID();

        for (int j = 0; j < world_obj.size(); j++)
        {
            //const collision_detection::CollisionGeometryData* collision_object = static_cast<const collision_detection::CollisionGeometryData*>(world_obj[j]->collisionGeometry()->getUserData());
            double dist = fcl::distance(robot_obj[i].get(), world_obj[j].get(), req, res);

            //ob_link.objects.push_back(collision_object->getID());
            ROS_INFO("Distance for %s: %f",robot_link->getID().c_str(), dist);
            //ob_link.distances.push_back(dist);
        }
        robot_link = NULL;
        delete robot_link;

        //ob.links.push_back(ob_link);*/
    }
    //obstacle_distance_publisher.publish(ob);
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

void ObstacleDistance::getRequestedLinks()
{
    std::string test;
    getParam("/test_config", test);

    XmlRpc::XmlRpcValue testblocks;

    if (getParam("obstacle_distance_node/test_config/" + test, testblocks))
    {
        std::map<std::string, XmlRpc::XmlRpcValue>::iterator i;
        for (i = testblocks.begin(); i != testblocks.end(); i++)
        {
            std::cout << i->first << " | " << i->second << std::endl;
        }
    }
}

ObstacleDistance::ObstacleDistance()
        : ros::NodeHandle()
{
    PLANNING_SCENE_SERVICE = "get_planning_scene";
    double publish_frequency = 10.0; //Hz
    ObstacleDistance::getRequestedLinks();

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_ =  boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_listener_);

    //Initialize timer & subscriber
    joint_state_subscriber = subscribe("joint_states", 1, &ObstacleDistance::joint_state_callback, this);
    obstacle_distance_publisher = advertise<atf_msgs::ObstacleDistance>("/atf/obstacle_distance", 1);
    obstacle_distance_timer = createTimer(ros::Duration(0.1), &ObstacleDistance::getDistanceToObstacles, this);

    //Initialize thread for updating the planning scene
    boost::thread* ps_thread = new boost::thread(&ObstacleDistance::getPlanningScene, this);
}

ObstacleDistance::~ObstacleDistance()
{}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "obstacle_distance");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
