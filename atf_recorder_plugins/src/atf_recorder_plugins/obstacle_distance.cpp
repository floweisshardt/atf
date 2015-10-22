#include <atf_recorder_plugins/obstacle_distance.h>

class CreateCollisionWorld : public collision_detection::CollisionWorldFCL {
public:
    CreateCollisionWorld(const collision_detection::WorldPtr &world) :
            CollisionWorldFCL(world) {
    }

    void getCollisionObject(std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj) {
        std::map<std::string, collision_detection::FCLObject>::iterator it = fcl_objs_.begin();
        obj.reserve(fcl_objs_.size());

        for (it; it != fcl_objs_.end(); ++it) {
            obj.push_back(it->second.collision_objects_[0]);
        }
    }
};

class CreateCollisionRobot : public collision_detection::CollisionRobotFCL {
public:
    CreateCollisionRobot(const robot_model::RobotModelConstPtr &model) :
            CollisionRobotFCL(model) {
    }

    void getCollisionObject(const robot_state::RobotState &state,
                            std::vector<boost::shared_ptr<fcl::CollisionObject> > &obj) {
        collision_detection::FCLObject fcl_obj;
        constructFCLObject(state, fcl_obj);
        obj = fcl_obj.collision_objects_;
    }
};

void ObstacleDistance::updatedScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type) {

    planning_scene_monitor::LockedPlanningSceneRO ps(planning_scene_monitor_);
    planning_scene::PlanningScenePtr planning_scene_ptr = ps->diff();

    CreateCollisionWorld collision_world(planning_scene_ptr->getWorldNonConst());
    robot_state::RobotState robot_state(planning_scene_ptr->getCurrentState());

    CreateCollisionRobot collision_robot(robot_state.getRobotModel());
    std::vector<boost::shared_ptr<fcl::CollisionObject> > robot_obj, world_obj;

    collision_robot.getCollisionObject(robot_state, robot_obj);
    collision_world.getCollisionObject(world_obj);

    atf_msgs::ObstacleDistance ob;

    for (int i = 0; i < robot_obj.size(); i++) {
        const collision_detection::CollisionGeometryData *robot_link =
                static_cast<const collision_detection::CollisionGeometryData *>(robot_obj[i]->collisionGeometry()->getUserData());

        atf_msgs::ObstacleDistanceLink ob_link;
        ob_link.name = robot_link->getID();

        for (int j = 0; j < world_obj.size(); j++) {
            const collision_detection::CollisionGeometryData *collision_object =
                    static_cast<const collision_detection::CollisionGeometryData *>(world_obj[j]->collisionGeometry()->getUserData());
            fcl::DistanceResult res;
            res.update(MAXIMAL_MINIMAL_DISTANCE, NULL, NULL, fcl::DistanceResult::NONE, fcl::DistanceResult::NONE);

            double dist = fcl::distance(robot_obj[i].get(), world_obj[j].get(), fcl::DistanceRequest(), res);
            if (dist < 0) {
                dist = 0;
            }

            ob_link.objects.push_back(collision_object->getID());
            ob_link.distances.push_back(dist);
        }
        ob.links.push_back(ob_link);
    }
    obstacle_distance_publisher_.publish(ob);
}

ObstacleDistance::ObstacleDistance()
        : ros::NodeHandle() {
    MAXIMAL_MINIMAL_DISTANCE = 5.0; //m
    double update_frequency = 100.0; //Hz
    bool error = false;

    std::string robot_description;
    std::vector<std::string> distance_topic;
    getParam(ros::this_node::getName() + "/obstacle_distance/robot_description", robot_description);
    getParam(ros::this_node::getName() + "/obstacle_distance/topics", distance_topic);
    if (robot_description == "" || distance_topic[0] == "") error = true;

    //Initialize planning scene monitor
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    try {
        planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description,
                                                                                                   tf_listener_);
    } catch (ros::InvalidNameException) {
        error = true;
    }

    obstacle_distance_publisher_ = advertise<atf_msgs::ObstacleDistance>(distance_topic[0], 1);

    if (!error) {
        planning_scene_monitor_->setStateUpdateFrequency(update_frequency);
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        planning_scene_monitor_->addUpdateCallback(boost::bind(&ObstacleDistance::updatedScene, this, _1));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_distance_node");

    ObstacleDistance ob;
    ros::spin();

    ros::shutdown();
    return 0;
}
