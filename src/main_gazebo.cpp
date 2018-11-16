///Trajectory Following for Jackal 15.0
/*
 * main function for jackal control
 * 2017.11.26
 * C++ Version: 2018.11.11
 */

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

// controller and traj generator packages
#include"Traj_generator.h"
#include"Jackal_controller.h"

using namespace std;


//read a document from the txt.file
void read_text(string &filename, vector<vector<double> > &path);


//main function
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "jackal_zhenghe");
    // ros::NodeHandle nh_control_jackal;
    ros::NodeHandle nh;

    // original path from text file
    vector<vector<double> > path; 

    // read original path from text file
    string name;
    ros::param::get("~param",name);
    read_text(name, path);
    cout << "Line 499 is ok!" << endl;

    // PART I: generate a trajectory with given path from the text file
    jackal_zhenghe::Traj_generator traj_generator = jackal_zhenghe::Traj_generator();    
    // Generated refined path with dense way-points and kinematic info
    vector<vector<double> > path_cont = traj_generator.generate_path(path); 
    // Generated Trajectory
    vector<vector<double> > traj = traj_generator.generate_traj(path_cont);
    cout << "Line 501 is ok!" << endl;

    // PART II: use the controller to subscribe kinematic feedback and publish control command
    cout << "Line 522 is ok!" << endl;
    ros::Publisher pub_control = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2000);
    cout << "Line 524 is ok!" << endl;
    jackal_zhenghe::Jackal_controller jackal_controller = jackal_zhenghe::Jackal_controller(pub_control, path, traj);  
    cout << "Line 526 is ok!" << endl;
    ros::Subscriber sub = nh.subscribe("/odometry/relative", 2000, &jackal_zhenghe::Jackal_controller::poseMessageRecieved_odom, &jackal_controller);

    ros::spin();
}


//read a document from the txt.file
void read_text(string &filename, vector<vector<double> > &path)
{
    ifstream file(filename.c_str());
    string  line;

    cout << "open file" << endl;

    // Read one line at a time into the variable line:
    while(getline(file, line))
    {
        vector<double>   lineData;
        stringstream  lineStream(line);

        double value;
        // Read an integer at a time from the line
        while(lineStream >> value)
        {
            // Add the integers from a line to a 1D array (vector)
            lineData.push_back(value);
        }
        // When all the integers have been read, add the 1D array
        // into a 2D array (as one line in the 2D array)
        path.push_back(lineData);
    }
}
