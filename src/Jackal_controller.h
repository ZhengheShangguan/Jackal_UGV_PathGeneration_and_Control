///Trajectory Following for Jackal 15.0
/*
 * include file for jackal control
 * 2017.11.26
 * C++ Version: 2018.11.11
 */

#ifndef JACKAL_CONTROLLER_H
#define JACKAL_CONTROLLER_H

// ros related packages
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

// some useful libraries in C++
#include <iostream>
#include <sstream>
#include <iterator>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

namespace jackal_zhenghe
{

class Jackal_controller
{

public:

    Jackal_controller(ros::Publisher pub_control, const std::vector<std::vector<double> > & path, const std::vector<std::vector<double> > & traj);

    // My controller for Trajectory Following
    void ugv_kinematic_controller();

    // Receive the current pose data from motion capture system
    void poseMessageRecieved_odom(const nav_msgs::Odometry & msg);


private:

    // HELPER FUNCTIONS: for ugv_kinematic_controller()
    // Helper Function: calculate the correct theta-error for the controller
    double cal_theta(double theta_des, double theta_curr);

    // Helper Function: calculate the distance2 between a points of vector type and a point of geometry_msgs::Pose2D
    double distance2(geometry_msgs::Pose2D pose1, std::vector<double> pt2);

    // Helper Function: calculate the distance3 between two points of geometry_msgs::Pose2D
    double distance3(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2);



    // HELPER FUNCTIONS: for poseMessageRecieved_odom()
    // Helper Function: find closest point to pt
    void find_closest_pt(geometry_msgs::Pose2D pose_curr);
    
    // Helper function: check if jackal pass the line to switch to next segment of traj
    bool pass_line(std::vector<double> & path_pt, geometry_msgs::Pose2D & this_pt, geometry_msgs::Pose2D & prev_pt);


private:

    // the same publisher as the one in main.function (because publisher is just a reference)
    ros::Publisher _pub_control_;

    // The path vector and the traj vector copy from the object Traj_generator
    std::vector<std::vector<double> > mpath;
    std::vector<std::vector<double> > mtraj;

    // current pose
    geometry_msgs::Pose2D pose_curr; 

    // last pose
    geometry_msgs::Pose2D pose_prev; 

    // desired pose
    geometry_msgs::Pose2D pose_des;

    // count the corresponding # of the original path way-point of current pose
    int _count_;

    // Most important variable: This represents the closest point to pose_curr
    std::vector<double> cls_pt;

    // control message
    geometry_msgs::Twist vel_cmd;

    // Smoothing the outputs of the controller
    double vel_prev;
    double omega_prev;

    // Initialization works: current position in the whole trajectory
    std::vector<std::vector<double> >::iterator it_traj;
};

} //namespace jackal_zhenghe

#endif // JACKAL_CONTROLLER_H