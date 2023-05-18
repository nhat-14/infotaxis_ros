#include <visual_cpt.h>
#include <tf/tf.h>

VisualCPT::VisualCPT(ros::NodeHandle *nh): nh_(nh) {
    switch_marker = nh_->advertise<visualization_msgs::Marker>("switch_marker", 10);
    hit_marker    = nh_->advertise<visualization_msgs::Marker>("hit_marker", 10);
    test_marker   = nh_->advertise<visualization_msgs::Marker>("test_marker", 10);
}   

VisualCPT::~VisualCPT(){}

visualization_msgs::Marker VisualCPT::emptyMarker(int numCells) {
    visualization_msgs::Marker points;
    points.header.frame_id="map";
    points.header.stamp=ros::Time::now();
    points.ns = "cells";
    points.id = 0;
    points.type=visualization_msgs::Marker::POINTS;
    points.action=visualization_msgs::Marker::ADD;

    Eigen::Vector3d colour = valueToColor(1.0/numCells, 0, 1);
    points.color.r = colour[0];
    points.color.g = colour[1];
    points.color.b = colour[2];
    points.color.a = 1.0;
    points.scale.x=0.15;
    points.scale.y=0.15;
    return points;
}


Eigen::Vector3d VisualCPT::valueToColor(double val, double low, double high){
    double r, g, b;
    val = log10(val);
    double range=(log10(high)-log10(low))/4;
    low=log10(low);
    if(val<low+range){
        r=0;
        g=std::max<double>((val-low)/(range),0);
        b=1;
    }
    else if(val<low+2*range){
        r=0;
        g=1;
        b=1-std::max<double>((val-(low+range))/(range),0);
    }
    else if(val<low+3*range){
        r=(val-(low+2*range))/(range);
        g=1;
        b=0;
    }else{
        r=1;
        g=1-std::max<double>(0,(val-(low+3*range))/(range));
        b=0;
    }
    return Eigen::Vector3d(r,g,b);
}

void VisualCPT::switch_notify(int planning_mode) {
    double r=0, g=1, b=0;   //green
    if (planning_mode == 1 || planning_mode == 2) {
        r=1; g=0; b=0;      //red
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.header.stamp=ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1.5;
    marker.pose.position.y = 2.4;
    marker.pose.position.z = 2.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    switch_marker.publish(marker);
}

void VisualCPT::hit_notify(bool gasHit) {
    double r=0, g=1, b=0;   //green
    if (gasHit == true) {
        r=1; g=0; b=0;      //red
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.header.stamp=ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3.1;
    marker.pose.position.y = 2.4;
    marker.pose.position.z = 2.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    hit_marker.publish(marker);
}

void VisualCPT::plotplot(float haha) {
    visualization_msgs::Marker wind_point_inv;
    wind_point_inv.header.frame_id = "map";
    wind_point_inv.id = 1;
	wind_point_inv.action = visualization_msgs::Marker::ADD;
	wind_point_inv.type = visualization_msgs::Marker::ARROW;
    wind_point_inv.header.stamp    = ros::Time::now();
	
    wind_point_inv.pose.position.x = 0.0;
	wind_point_inv.pose.position.y = 0.0;
    wind_point_inv.pose.position.z = 0.0;

    wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(haha);
    wind_point_inv.scale.x = 1.5;	  //arrow leng`ht
    wind_point_inv.scale.y = 0.1;	  //arrow width
    wind_point_inv.scale.z = 0.1;	  //arrow height
    wind_point_inv.color.r = 0.0;
    wind_point_inv.color.g = 0.0;
    wind_point_inv.color.b = 1.0;
    wind_point_inv.color.a = 1.0;

    test_marker.publish(wind_point_inv);
}