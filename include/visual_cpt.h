#ifndef VISUAL_CPT
#define VISUAL_CPT
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>

class VisualCPT {
    public:
        VisualCPT(ros::NodeHandle *nh);
        ~VisualCPT();
        
        void switch_notify(int planning_mode);
        void hit_notify(bool gasHit);
        void plotplot(float haha);
        Eigen::Vector3d valueToColor(double val, double low, double high);
        visualization_msgs::Marker emptyMarker(int numCells);
     
    protected:
        ros::NodeHandle *nh_;
        ros::Publisher switch_marker;
        ros::Publisher hit_marker;
        ros::Publisher test_marker;
};
#endif