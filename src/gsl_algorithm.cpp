#include <gsl_algorithm.h>
#include <boost/format.hpp>

GSLAlgorithm::GSLAlgorithm(ros::NodeHandle *nh) : nh_(nh), mb_ac("move_base", true) {
    srand(time(NULL));      //initialize random seed
    ROS_INFO("Waiting for the move_base action server...");
    bool mb_aconline = false;
    for(int i=0 ; i<10 ; i++) {
        if(mb_ac.waitForServer(ros::Duration(1.0))) {
            mb_aconline = true;
            break;
        }
        ROS_INFO("Unable to find the move_base action server, retrying...");
    }
    if(!mb_aconline) {
        ROS_FATAL("No move_base node found. Please ensure the move_base node is active.");
        ROS_BREAK();
        return;
    }
    ROS_INFO("Found MoveBase! Initializing module...");

    //===================== Load Parameters ===================
    nh->param<std::string>("enose_topic", enose_topic, "/PID/Sensor_reading");
    nh->param<std::string>("anemometer_topic", anemometer_topic, "/Anemometer/WindSensor_reading");
    nh->param<std::string>("robot_location_topic", robot_location_topic, "/amcl_pose");
    nh->param<std::string>("map_topic", map_topic, "/map");

    nh->param<double>("max_search_time", max_search_time, 500.0);
    nh->param<double>("distance_found", distance_found, 0.5);
    nh->param<double>("ground_truth_x", source_pose_x, 1.5);
    nh->param<double>("ground_truth_y", source_pose_y, 3.0);
    nh->param<double>("robot_pose_x", robot_pose_x, 0.0);
    nh->param<double>("robot_pose_y", robot_pose_y, 0.0);

    //====================== Subscribers ======================
    localization_sub_ = nh_->subscribe(robot_location_topic,100,&GSLAlgorithm::localizationCallback,this);

    //======================= Services ========================
    mb_client = nh_->serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    inMotion = false;
    inExecution = false;
}

GSLAlgorithm::~GSLAlgorithm(){}


//=================================================================================
//                                 Callback
//=================================================================================

void GSLAlgorithm::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    current_robot_pose = *msg;  //keep the most recent robot pose
    robot_poses_vector.push_back(current_robot_pose); //Keep all poses
}


// Move Base CallBacks
void GSLAlgorithm::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result) {
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_DEBUG("PlumeTracking - %s - Target achieved!", __FUNCTION__);
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_DEBUG("PlumeTracking - %s - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}

void GSLAlgorithm::goalActiveCallback(){}
void GSLAlgorithm::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}


//===================================================================
//                            AUX
//====================================================================
bool GSLAlgorithm::get_inMotion() {
    return inMotion;
}

float GSLAlgorithm::get_average_vector(std::vector<float> const &v) {
    size_t length = v.size();
    float sum = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;
    return sum/length;
}


bool GSLAlgorithm::checkGoal(move_base_msgs::MoveBaseGoal * goal) {
    geometry_msgs::PoseStamped target = goal->target_pose;
    float target_x = target.pose.position.x;
    float target_y = target.pose.position.y;

    ROS_INFO("Checking Goal [%.2f, %.2f] in map frame", target_x, target_y);

    //1. Get dimensions of OccupancyMap
    float map_min_x = map_.info.origin.position.x;
    float map_max_x = map_.info.origin.position.x + map_.info.width*map_.info.resolution;
    float map_min_y = map_.info.origin.position.y;
    float map_max_y = map_.info.origin.position.y + map_.info.height*map_.info.resolution;

    //2. Check that goal falls inside the map
    if (target_x < map_min_x || target_x > map_max_x || target_y < map_min_y || target_y > map_max_y) {
        ROS_ERROR("Goal is out of map dimensions!!!");
        return false;
    }

    //3. Use Move Base Service to declare a valid navigation goal
    nav_msgs::GetPlan mb_srv;
    geometry_msgs::PoseStamped start_point;

    start_point.header.frame_id = "map";
    start_point.header.stamp = ros::Time::now();
    start_point.pose = current_robot_pose.pose.pose;

    mb_srv.request.start = start_point;
    mb_srv.request.goal = target;
    mb_srv.request.tolerance = 0.1;
    
    //get path from robot to next target.
    mb_client.call(mb_srv);
    if( mb_client.call(mb_srv) && mb_srv.response.plan.poses.size()>1) {
        return true;
    }
    else {
        ROS_ERROR("Unable to reach [%.2f, %.2f] with MoveBase", target_x, target_y);
        return false;
    }
}


int GSLAlgorithm::checkSourceFound() {
    if (inExecution) {
        //Check if timeout
        ros::Duration time_spent = ros::Time::now() - start_time;
        if (time_spent.toSec() > max_search_time) {
            ROS_INFO("FAILURE-> Time spent (%.3f s) > max_search_time = %.3f", time_spent.toSec(), max_search_time);
            return 0;
        }

        //Check the distance from robot to source
        double Ax = current_robot_pose.pose.pose.position.x - source_pose_x;
        double Ay = current_robot_pose.pose.pose.position.y - source_pose_y;
        double dist = sqrt(pow(Ax,2) + pow(Ay,2));  
        if (dist < distance_found) {
            ROS_INFO("SUCCESS -> Time spent (%.3f s)", time_spent.toSec());
            return 1;
        }
    }
    //In other case, keep searching
    return -1;
}