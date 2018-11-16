///Trajectory Following for Jackal 15.0
/*
 * src file for jackal traj_generator
 * 2017.11.26
 * C++ Version: 2018.11.11
 */

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

#include"Traj_generator.h"

// Define some constants
#define PI   3.14159265358979323846
#define v_const 0.4
#define t_const 0.375
#define dist_const 0.15
#define pt_num 20
#define omega_const 0.4
#define vel_max 0.45
#define vel_min 0.10

using namespace std;

namespace jackal_zhenghe
{
    Traj_generator::Traj_generator(): forward_mode(1)
    {}


    // generate_path HELPER Function: forward mode
    void Traj_generator::compute_mode(vector<double> ptl, vector<double> ptr)
    {
        // compute mode
        double x_ = ptl[0] - ptr[0];
        double y_ = ptl[1] - ptr[1];
        double _theta_ = ptl[2];
        double alpha = -_theta_ + atan2f(-y_,-x_); 

        if (alpha > PI)
            alpha = alpha - 2*PI;
        else if (alpha < -PI)
            alpha = alpha + 2*PI; 

        if (alpha > -PI/2 && alpha <= PI/2)
            forward_mode = 1;
        else
            forward_mode = -1;
    }


    // generate_path HELPER Function: calculate the distance between two points of vector type
    double Traj_generator::distance_(vector<double> pt1, vector<double> pt2)
    {
        return sqrt((pt2[0] - pt1[0])*(pt2[0] - pt1[0]) + (pt2[1] - pt1[1])*(pt2[1] - pt1[1]));
    }

    // generate_path HELPER Function: set corret rho for path planning with given virtual velocity and acceleration
    double Traj_generator::set_pathrho(double vel_x, double vel_y, double acc_x, double acc_y)
    {
        //denominator for calculating rho
        double den_rho;
        //local variable -- rho
        double rho;

        if (abs(vel_x) < 0.000001)
        {
            rho = 10000;
        }
        else
        {
            den_rho = (acc_y * vel_x - acc_x * vel_y) / (vel_x * vel_x * vel_x);
            if (abs(den_rho) < 0.0001)
                rho = 10000;
            else
                rho = sqrt((1+(vel_y/vel_x)*(vel_y/vel_x))*(1+(vel_y/vel_x)*(vel_y/vel_x))*(1+(vel_y/vel_x)*(vel_y/vel_x))) / den_rho;        
        }
        return rho;
    }


    // generate_path HELPER Function: set correct angle for path planning with the range (-pi,pi)
    double Traj_generator::set_pathangle(double vel_y, double vel_x, int forward_mode)
    {
        double angle_correct = atan2(vel_y, vel_x);

        if (forward_mode == -1)
        {
            if (angle_correct <= 0 && angle_correct >= -PI)
                angle_correct += PI;
            else if (angle_correct > 0 && angle_correct <= PI)
                angle_correct -= PI;
        }

        return angle_correct;
    }


    // generate_path Function
    vector<vector<double> > Traj_generator::generate_path(vector<vector<double> >& path)
    {
        // public containers
        vector<vector<double> > path_cont;
        vector<double> pose_path; // container for all information a single way-point
        int count_path = -1; // # of way-point on the path
        double rho_new; // the curvature with a sign indicating the side of its center 

        //generate a path with those given way-points
        for (vector<vector<double> >::iterator it_path = path.begin(), it_path_end = path.end() - 1; it_path != it_path_end; it_path++)
        {
            //dereference the iterators to well define left and right points first
            vector<double> ptl_path = *(it_path);
            vector<double> ptr_path = *(it_path+1);

            // Here, ignore the inconsistency of vel0_x and vel0_y when the car needs to stop
            // at a point then go back. Cuz we only need the path, not the trajectory here.
            compute_mode(ptl_path, ptr_path);

            //calculate all related Kinematics infomation for each way-point on the path
            //PS: There is no relation between forward_mode and pose.theta, the former refers to the backward or
            //forward mode, the latter refers to the direction of the jackal in 2-D plane from -pi to pi.
            double vel0_x = forward_mode * v_const * cos(ptl_path[2]);
            double vel0_y = forward_mode * v_const * sin(ptl_path[2]);
            double vel1_x = forward_mode * v_const * cos(ptr_path[2]);
            double vel1_y = forward_mode * v_const * sin(ptr_path[2]);

            //set a unit time variable t_meta and a temporary time variable t_curr for the following calculations
            double dist = distance_(ptl_path, ptr_path);
            double t_tol = t_const * dist / dist_const;
            double t_meta = t_tol / pt_num;
            double t_curr = 0;

            //before pushback, add 1 to the count_path to switch to next range of the trajectory
            count_path++;

            //coefficients for convenience
            double coeff01 = 3*ptr_path[0] - 3*ptl_path[0] - 2*vel0_x*t_tol - vel1_x*t_tol;
            double coeff02 = 2*ptl_path[0] - 2*ptr_path[0] + vel0_x*t_tol + vel1_x*t_tol;
            double coeff11 = 3*ptr_path[1] - 3*ptl_path[1] - 2*vel0_y*t_tol - vel1_y*t_tol;
            double coeff12 = 2*ptl_path[1] - 2*ptr_path[1] + vel0_y*t_tol + vel1_y*t_tol;

            //push_back the start point into the traj first, then begin the for-loop
            //adjust the very original way-point's theta to be in the range (-pi, pi)
            while (ptl_path[2] > PI)
            ptl_path[2] -= 2*PI;
            while (ptl_path[2] < -PI)
            ptl_path[2] += 2*PI;
            rho_new = set_pathrho(vel0_x, vel0_y, 2*coeff01/(t_tol*t_tol), 2*coeff11/(t_tol*t_tol));
            pose_path.push_back(ptl_path[0]);
            pose_path.push_back(ptl_path[1]);
            pose_path.push_back(ptl_path[2]);
            pose_path.push_back(rho_new);
            pose_path.push_back(count_path);
            pose_path.push_back(forward_mode);
            path_cont.push_back(pose_path);
            pose_path.clear();
            cout << fixed <<"Start Point: ptl_path[0]:"<<ptl_path[0]<<", ptl_path[1]: "<<ptl_path[1]<<", ptl_path[2]: "<<ptl_path[2]<<", rho:"<<rho_new<<endl;

            for (int i = 1; i < pt_num; i++)
            {
                double arr_path[6] = {};

                //For this point in this small sketch of the path
                t_curr += t_meta;

                //Use a reinitialized array to store every point's kinematic infomation
                arr_path[0] = ptl_path[0] + vel0_x*t_curr + coeff01 * (t_curr/t_tol) * (t_curr/t_tol) + coeff02 * (t_curr/t_tol) * (t_curr/t_tol) * (t_curr/t_tol);
                arr_path[1] = ptl_path[1] + vel0_y*t_curr + coeff11 * (t_curr/t_tol) * (t_curr/t_tol) + coeff12 * (t_curr/t_tol) * (t_curr/t_tol) * (t_curr/t_tol);
                double vel_x = vel0_x + 2*t_curr*coeff01/(t_tol*t_tol) + 3*t_curr*t_curr*coeff02/(t_tol*t_tol*t_tol);
                double vel_y = vel0_y + 2*t_curr*coeff11/(t_tol*t_tol) + 3*t_curr*t_curr*coeff12/(t_tol*t_tol*t_tol);
                double acc_x = 2*coeff01/(t_tol*t_tol)+6*t_curr*coeff02/(t_tol*t_tol*t_tol);
                double acc_y = 2*coeff11/(t_tol*t_tol)+6*t_curr*coeff12/(t_tol*t_tol*t_tol);
                arr_path[2] = set_pathangle(vel_y, vel_x, forward_mode);
                arr_path[3] = set_pathrho(vel_x, vel_y, acc_x, acc_y);
                arr_path[4] = count_path;
                arr_path[5] = forward_mode;
                cout << "vel_x = " << vel_x << endl;
                cout << "vel_y = " << vel_y << endl;

                // When the calculation of the 1D vector arr_path is accomplished,
                // add it into pose_path, then into my continuous path
                pose_path.insert(pose_path.begin(), arr_path, arr_path + 6);
                path_cont.push_back(pose_path);
                //clear the array for next point
                pose_path.clear();
            }
        }

        //we still need path.end() -1, the end point in the trajectory
        vector<double> path_end = *(path.end() -1);
        count_path++;

        //push back all the information of end point into the pose_path vector
        pose_path.push_back(path_end[0]);
        pose_path.push_back(path_end[1]);
        pose_path.push_back(path_end[2]);
        pose_path.push_back(rho_new);
        pose_path.push_back(count_path);
        pose_path.push_back(forward_mode);  
        path_cont.push_back(pose_path);

        // //write this huge vector into a .txt file
        // ofstream pathdata;
        // pathdata.open("path_data.txt", fstream::app);
        // for (it = path_cont.begin(); it != (path_cont.end()); it++)
        // {
        //     pathdata << fixed << (*(it))[0] << " ";
        //     pathdata << fixed << (*(it))[1] << " ";
        //     pathdata << fixed << (*(it))[2] << " ";
        //     pathdata << fixed << (*(it))[3] << " ";
        //     pathdata << fixed << (*(it))[4] << " ";
        //     pathdata << fixed << (*(it))[5] << endl;
        // }
        // pathdata.close();
        // cout << "path_data.txt is saved!" << endl;

        return path_cont;
    }


    // generate_traj Function: based on generated refined path
    vector<vector<double> > Traj_generator::generate_traj(vector<vector<double> >& path_cont)
    {
        // some containers
        vector<vector<double> > traj;
        vector<double> pose_new;
        //nominal linear velocity is based on the curvature at this point
        double vel_tol;
        //nominal angular velocity
        double omega_n;
        //initial velocity for deceleration period
        double vel_init;

        // Traj 01: Gentle Start Period
        // Initialization stage
        vector<vector<double> >::iterator it = path_cont.begin();
        vel_tol = (*it)[5] * 0.03;
        // Accelerate until it reaches the nominal velocity determined by the current curvature
        while (((abs(vel_tol) < abs((*it)[3] * omega_const)) && (abs(vel_tol) < vel_max)) || (abs(vel_tol) < vel_min))
        {
            //push_back the start point into the trajectory first, then begin the for-loop
            for (int i=0; i<6; ++i)
                pose_new.push_back((*it)[i]);

            // determine the value of omega_n
            omega_n = vel_tol / ((*it)[3]);
            if ((*it)[5] * ((*it)[0]-(*(it+1))[0]) >= 0)
                omega_n *= -1;

            //push the velocity infomation and clear the pose_new vector for next point    
            pose_new.push_back(vel_tol);
            pose_new.push_back(omega_n);
            traj.push_back(pose_new);
            pose_new.clear();

            //update the iterator and the velocity with the calculated maximal delta_vel
            it++;
            vel_tol += (0.00375 / vel_tol);
        }

        // Traj 02: Middle Period, the backmode taken into account
        while (it != path_cont.end()-40)
        {
            // i. Detect the mode switch, and give the complete velocity procedure if there is a switch ahead
            if ((*(it+39))[5] * (*(it+40))[5] == -1)
            {
                //deceleration period first, until jackal stop at the exact mode-switch way-point
                {
                    vel_init = vel_tol;
                    for(int ct = 0; ct < 40; ct++)
                    {
                        //push_back the start point into the trajectory first, then begin the for-loop
                        for (int i=0; i<6; ++i)
                            pose_new.push_back((*it)[i]);
                    
                        // determine the value of linear velocity: calculate a with v1^2-0=2*a*x, then find delta_velo with v1^2-v2^2=2*a*x0 
                        vel_tol -= 0.0076 * (vel_init*vel_init/0.6) / vel_tol;
                        omega_n = vel_tol / (*it)[3];
                        if (((*it)[5]) * ((*it)[0]-(*(it+1))[0]) >= 0)
                            omega_n *= -1;
                    
                        //push the velocity infomation and clear the pose_new vector for next point    
                        pose_new.push_back(vel_tol);
                        pose_new.push_back(omega_n);
                        traj.push_back(pose_new);
                        pose_new.clear();

                        //update the iterator and the velocity with the calculated maximal delta_vel
                        it++;
                    }
                }
                //stop and restart point, *it is already (it+40) now
                {
                    for (int i=0; i<6; ++i)
                        pose_new.push_back((*it)[i]);

                    vel_tol = 0;
                    omega_n = 0;
                    pose_new.push_back(0); // make sure it passes the cross-line and switch to the next trajectory
                    pose_new.push_back(0);
                    traj.push_back(pose_new);
                    //clear the pose_new vector for last point
                    pose_new.clear();
                    it++;
                }
                //restart period until it reaches the normal speed
                {
                    //give the initialized linear velocity
                    vel_tol = (*it)[5] * 0.03;
                    while (((abs(vel_tol) < abs((*it)[3] * omega_const)) && (abs(vel_tol) < vel_max)) || (abs(vel_tol) < vel_min))
                    {
                        //push_back the start point into the trajectory first, then begin the for-loop
                        for (int i=0; i<6; ++i)
                            pose_new.push_back((*it)[i]);
                    
                        // determine the value of omega_n
                        omega_n = vel_tol / ((*it)[3]);
                        if ((*it)[5] * ((*it)[0]-(*(it+1))[0]) >= 0)
                            omega_n *= -1;
                    
                        //push the velocity infomation and clear the pose_new vector for next point    
                        pose_new.push_back(vel_tol);
                        pose_new.push_back(omega_n);
                        traj.push_back(pose_new);
                        pose_new.clear();
                    
                        //update the iterator and the velocity with the calculated maximal delta_vel
                        it++;
                        vel_tol += (0.00375 / vel_tol);
                    }
                }
            }

            // ii. Give the correct velocity if there is no mode switch around.
            //Calculate the corresponding forward_mode to identify the values for Vn and Wn
            else
            {
                //push_back the start point into the trajectory first, then begin the for-loop
                for (int i=0; i<6; ++i)
                    pose_new.push_back((*it)[i]);

                //define nominal velocity and nominal angular velocity
                if (abs(omega_const * ((*it)[3])) > vel_max)
                {
                    omega_n = ((*it)[5]) * vel_max / ((*it)[3]);
                    if (((*it)[5]) * ((*it)[0]-(*(it+1))[0]) >= 0)
                        omega_n *= -1;
                    vel_tol = ((*it)[5]) * vel_max;
                }
                else if (abs(omega_const*((*it)[3])) < vel_min)
                {
                    //let omega to be the limit value for it, which is 0.5, let controller decide what to add or reduce
                    omega_n = ((*it)[5]) * (((*it)[3]>0) ? 1:-1) * 0.5;
                    if (((*it)[5]) * ((*it)[0]-(*(it+1))[0]) >= 0)
                        omega_n *= -1;
                    vel_tol = ((*it)[5]) * vel_min;
                }
                else
                {
                    vel_tol = ((*it)[5]) * abs((omega_const)*((*it)[3]));
                    omega_n = vel_tol / ((*it)[3]);
                    if (((*it)[5]) * ((*it)[0]-(*(it+1))[0]) >= 0)
                        omega_n *= -1;
                }
                //push the nominal velocity and clear the pose_new vector for next point
                pose_new.push_back(vel_tol);
                pose_new.push_back(omega_n);
                traj.push_back(pose_new);
                pose_new.clear();

                // update the iterator in the end of the while loop
                it++;      
            }
        }

        // Traj 03: Deceleration Period
        // denote the beginning velocity as vel_init
        vel_init = vel_tol;
        while (it != path_cont.end()-1)
        {
            //push_back the start point into the trajectory first, then begin the for-loop
            for (int i=0; i<6; ++i)
                pose_new.push_back((*it)[i]);

            // determine the value of linear velocity: calculate a with v1^2-0=2*a*x, then find delta_velo with v1^2-v2^2=2*a*x0 
            vel_tol -= 0.0078 * (vel_init*vel_init/0.6) / vel_tol;
            omega_n = vel_tol / (*it)[3];
            if (((*it)[5]) * ((*it)[0]-(*(it+1))[0]) >= 0)
                omega_n *= -1;

            //push the velocity infomation and clear the pose_new vector for next point    
            pose_new.push_back(vel_tol);
            pose_new.push_back(omega_n);
            traj.push_back(pose_new);
            pose_new.clear();

            //update the iterator and the velocity with the calculated maximal delta_vel
            it++;
        }

        // Traj 04: Last point for the trajectory
        {
            it = path_cont.end()-1;
            for (int i=0; i<6; ++i)
                pose_new.push_back((*it)[i]);

            //for last point
            pose_new.push_back(0);
            pose_new.push_back(0);
            traj.push_back(pose_new);
            //clear the pose_new vector for last point
            pose_new.clear();    
        }

        // //write this huge vector into a .txt file
        // ofstream trajdata;
        // trajdata.open("traj_data.txt", fstream::app);
        // for (it = traj.begin(); it != (traj.end()); it++)
        // {
        //     trajdata << fixed << (*(it))[0] << " ";
        //     trajdata << fixed << (*(it))[1] << " ";
        //     trajdata << fixed << (*(it))[2] << " ";
        //     trajdata << fixed << (*(it))[3] << " ";
        //     trajdata << fixed << (*(it))[4] << " ";
        //     trajdata << fixed << (*(it))[5] << " ";
        //     trajdata << fixed << (*(it))[6] << " ";
        //     trajdata << fixed << (*(it))[7] << endl;
        // }
        // trajdata.close();
        // cout << "traj_data.txt is saved!" << endl;

        return traj;
    }
    
} //namespace jackal_zhenghe