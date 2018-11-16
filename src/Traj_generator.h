///Trajectory Following for Jackal 15.0
/*
 * include file for jackal traj_generator
 * 2017.11.26
 * C++ Version: 2018.11.11
 */

#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

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

class Traj_generator
{

public:

    Traj_generator();

    std::vector<std::vector<double> > generate_path(std::vector<std::vector<double> >& path);

    std::vector<std::vector<double> > generate_traj(std::vector<std::vector<double> >& path_cont);


public:

    //Mode: forward = 1; backward = -1
    int forward_mode;


private:

    // HELPER FUNCTIONS: for generate_path
    //Function: forward mode
    void compute_mode(std::vector<double> ptl, std::vector<double> ptr);
    //Function: calculate the distance between two points of vector type
    double distance_(std::vector<double> pt1, std::vector<double> pt2);
    //Function: set corret rho for path planning with given virtual velocity and acceleration
    double set_pathrho(double vel_x, double vel_y, double acc_x, double acc_y);
    //Function: set correct angle for path planning with the range (-pi,pi)
    double set_pathangle(double vel_y, double vel_x, int forward_mode);

};

} //namespace jackal_zhenghe

#endif // TRAJ_GENERATOR_H