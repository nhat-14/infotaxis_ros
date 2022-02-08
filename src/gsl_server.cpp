#include "gsl_server.h"
#include "gsl_infotaxis.h"

//================================== CPT ALGORITHMS ===================================
int CGSLServer::doInfotaxis() {
    ros::NodeHandle nh("~");
    InfotaxisGSL infotatic(&nh);
    ros::Rate loop_rate(10);
    while(ros::ok() && infotatic.checkSourceFound() == -1) {
        ros::spinOnce();
        if(!infotatic.get_inMotion()) {
            switch(infotatic.getState()){
                case Infotaxis_state::WAITING_FOR_MAP:
                    ROS_INFO("[INFOTAXIS] Waiting for the map of the environment!...."); 
                    break;
                case Infotaxis_state::STOP_AND_MEASURE:
                    infotatic.getGasWindObservations();
                    break;
                case Infotaxis_state::MOVING:
                    infotatic.setGoal();
                    break;
                default:
                    ROS_ERROR("[INFOTAXIS] Search state is undefined!");
            }
        }
        loop_rate.sleep();
    }
    return infotatic.checkSourceFound();
}

//===================== Action Server Callback when Goal is received ====================
void CGSLServer::executeCB(const gsl_actionserver::gsl_action_msgGoalConstPtr &goal) {
    int is_found = doInfotaxis(); // res: 0(fail) 2(cancelled) 1(sucess)

    if (is_found == 1) { 
        result_.success = 1;
        ROS_INFO("[GSL_server]: Found the emission source!! ");
    }
    else if (is_found == 2) { // Canceled/Preempted by user?
        result_.success = 0;
        ROS_INFO("[GSL_server]: Action cancelled/preemted");
    }
    else {
        result_.success = 0;
        ROS_INFO("[GSL_server]: Couldn't find the gas source! SORRY!");
    }
    as_.setSucceeded(result_);
    ros::shutdown();
}


//================================== MAIN ===========================================

int main(int argc, char** argv) {
    ros::init(argc,argv,"gsl_ac");
    CGSLServer gsl_node("gsl");
    ROS_INFO("[GSL_server] GSL action server is ready for action!...");
    ros::spin();
    return 0;
}