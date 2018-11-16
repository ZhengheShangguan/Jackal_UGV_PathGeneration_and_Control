///Trajectory Following for Jackal 15.0
/*
 * src file for jackal control
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

#include"Jackal_controller.h"

//Constants
#define PI   3.14159265358979323846
#define pt_num 20
#define rad_ 0.03
//Lyapunov-based nonlinear controller _ parameters
#define Kx 10 //1.0
#define Ky 64 //6.4
#define Ktheta 16 //1.6
//Smoothing the controller outputs
#define vel_thre 0.01
#define omega_thre 0.1

using namespace std;

namespace jackal_zhenghe
{
    Jackal_controller::Jackal_controller(ros::Publisher pub_control, const vector<vector<double> > & path, const vector<vector<double> > & traj):
        _pub_control_(pub_control), mpath(path), mtraj(traj), _count_(0), vel_prev(0), omega_prev(0)
    {
        it_traj = mtraj.begin();
    }

    // HELPER FUNCTIONS: for ugv_kinematic_controller()
    
    // Helper Function: calculate the correct theta-error for the controller
    double Jackal_controller::cal_theta(double theta_des, double theta_curr)
    {
        double delta_angle = theta_des - theta_curr;

        //Usually the above is good, except the difference is larger than PI or smaller than -PI
        while (abs(delta_angle) > PI)
        {
            if (delta_angle > 0)
            delta_angle -= 2*PI;
            else if (delta_angle < 0)
            delta_angle += 2*PI;    
        }
            
        return delta_angle;
    }

    // Helper Function: calculate the distance2 between a points of vector type and a point of geometry_msgs::Pose2D
    double Jackal_controller::distance2(geometry_msgs::Pose2D pose1, vector<double> pt2)
    {
        return sqrt((pt2[0] - pose1.x)*(pt2[0] - pose1.x) + (pt2[1] - pose1.y)*(pt2[1] - pose1.y));
    }

    // Helper Function: calculate the distance3 between two points of geometry_msgs::Pose2D
    double Jackal_controller::distance3(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2)
    {
        return sqrt((pose2.x - pose1.x)*(pose2.x - pose1.x) + (pose2.y - pose1.y)*(pose2.y - pose1.y));
    }

    // My controller for Trajectory Following
    void Jackal_controller::ugv_kinematic_controller()
    {
        //for the new controller
        double x_e;
        double y_e;
        double delta_theta;
        
        //stop the controller when it reaches the destination
        if((_count_ >= (int)mpath.size()-1) && (distance2(pose_curr, *(mtraj.end()-1)) <= rad_))
        {
            //define the value of final theta pose for some judgement usage in the controller
            double theta_final = (*(mpath.end() -1))[2];
            while (theta_final > PI)
                theta_final -= 2*PI;
            while (theta_final < -PI)
                theta_final += 2*PI;
            if(abs(pose_curr.theta-theta_final) < 0.1)
            {
                vel_cmd.angular.z = 0;
                vel_cmd.linear.x = 0;                
            }
        }
        else
        {
            //start the controller
            cout << "Line: start the controller!" << endl;    
            
            //calculate some x-error, y-error for linear velocity control law
            x_e = cos(pose_curr.theta)*(cls_pt[0]-pose_curr.x) + sin(pose_curr.theta)*(cls_pt[1]-pose_curr.y);
            y_e = -sin(pose_curr.theta)*(cls_pt[0]-pose_curr.x) + cos(pose_curr.theta)*(cls_pt[1]-pose_curr.y);

            //calculate correct theta error
            delta_theta = cal_theta(cls_pt[2], pose_curr.theta);

            //Final control law
            //This is for the very start period, since the cls_pt strategy may cause the control law to 
            //intervene the start acceleration period, thus we waive the control w.r.t. the first way-point
            if ((_count_ == 0)||(_count_ == (int)mpath.size()))
            {
                vel_cmd.linear.x = cls_pt[6] * cos(delta_theta) + Kx/10 * x_e;
                vel_cmd.angular.z = cls_pt[7] + cls_pt[6] * (Ky/10 * y_e + cls_pt[5] * Ktheta/10*sin(delta_theta));
            }
            else
            {
                vel_cmd.linear.x = cls_pt[6] * cos(delta_theta) + Kx * x_e;
                vel_cmd.angular.z = cls_pt[7] + cls_pt[6] * (Ky * y_e + cls_pt[5] * Ktheta*sin(delta_theta));       
            }
            
            
            //Limiting the increment of linear velocity and angular velocity
            if (cls_pt[4]>=1)
            {
                if (vel_cmd.linear.x - vel_prev > vel_thre)
                    vel_cmd.linear.x = vel_prev + vel_thre;
                else if (vel_cmd.linear.x - vel_prev < -vel_thre)
                    vel_cmd.linear.x = vel_prev - vel_thre;

                if (vel_cmd.angular.z - omega_prev > omega_thre)
                    vel_cmd.angular.z = omega_prev + omega_thre;
                else if (vel_cmd.angular.z - omega_prev < -omega_thre)
                    vel_cmd.angular.z = omega_prev - omega_thre;
            }
            vel_prev = vel_cmd.linear.x;
            omega_prev = vel_cmd.angular.z;


            //For safety
            if (vel_cmd.linear.x > 1)
                vel_cmd.linear.x = 1; 
            else if (vel_cmd.linear.x < -1)
                vel_cmd.linear.x = -1;

            if (vel_cmd.angular.z > 0.7)
                vel_cmd.angular.z = 0.7;
            else if (vel_cmd.angular.z < -0.7)
                vel_cmd.angular.z = -0.7;

            

            // //Addition part: save the GLOBAL x_err, y_err, theta_err into corresponding .txt
            // ofstream errdata;
            // errdata.open("err_data.txt", fstream::app);
            // errdata << count__ << " ";
            // errdata << cls_pt[0]-pose_curr.x << " ";
            // errdata << cls_pt[1]-pose_curr.y << " ";
            // errdata << delta_theta <<endl;
            // errdata.close();
            // count__++;
            // cout << "err_data.txt is saved!" << endl;

            // //Addition part: save the current pose into a vector of vector curr_pose
            // //output the current condition (x,y,theta,vx,vy) into a .txt file
            // ofstream outdata;
            // outdata.open("curr_posedata.txt", fstream::app);
            // outdata << pose_curr.x << " ";
            // outdata << pose_curr.y << " ";
            // outdata << pose_curr.theta << " ";
            // outdata << vel_cmd.linear.x * cos(pose_curr.theta) << " ";
            // outdata << vel_cmd.linear.x * sin(pose_curr.theta) << " ";
            // outdata << vel_cmd.linear.x << " ";
            // outdata << vel_cmd.angular.z << " ";
            // outdata << count__-1 << " ";
            // outdata << delta_theta << " "; 
            // outdata << passline << " "; 
            // outdata << (*it_traj)[4] << endl;
            // outdata.close();s
            // cout << "Line 489 is ok!" << endl;

            double diff = pose_curr.theta - pose_des.theta;
            if(diff>PI)
            diff=diff-2*PI;
            else if(diff<-PI)
            diff=diff+2*PI;

        }
    }


    // HELPER FUNCTIONS: for poseMessageRecieved_odom()

    // Helper Function: find closest point to pt
    void Jackal_controller::find_closest_pt(geometry_msgs::Pose2D pose_curr)
    {
        //PS: _count_ is a globle variable to remind all related functions 
        //which way-point's corresponding trajectory we are following now
        cls_pt = *it_traj;
        vector<vector<double> >::iterator it_temp = it_traj;

        while (_count_ == (*it_temp)[4])
        {
            if (distance2(pose_curr, cls_pt) > distance2(pose_curr, *it_temp))
                cls_pt = *(it_temp);
            it_temp++;
        }
        if (distance2(pose_curr, cls_pt) > distance2(pose_curr, *(it_traj+pt_num)))
            cls_pt = *(it_traj+pt_num);
        //The next swich may still have a little problem, since when jackal departs the range 0.007, it may 
        //switch back the that restart way-point, which velocities are both zero.
        else if ((abs(cls_pt[6]) < 0.01) && (distance2(pose_curr, cls_pt) < 0.007))
            cls_pt = *(it_temp+1);
    }

    // Helper function: check if jackal pass the line to switch to next segment of traj
    bool Jackal_controller::pass_line(vector<double> & path_pt, geometry_msgs::Pose2D & this_pt, geometry_msgs::Pose2D & prev_pt)
    {
        int sign_1;
        int sign_2;

        if (this_pt.x+tan(path_pt[2])*this_pt.y-path_pt[0]-path_pt[1]*tan(path_pt[2]) > 0)
            sign_1 = 1;
        else
            sign_1 = -1;
        if (prev_pt.x+tan(path_pt[2])*prev_pt.y-path_pt[0]-path_pt[1]*tan(path_pt[2]) > 0)
            sign_2 = 1;
        else
            sign_2 = -1;
        
        if (sign_1 == sign_2)
            return false;
        else
            return true;
    }


    // Receive the current pose data from motion capture system
    void Jackal_controller::poseMessageRecieved_odom(const nav_msgs::Odometry & msg)
    {
        if (it_traj != (mtraj.end()-1))
        {
            cout << "Line 461 is good!" << endl;
            //First, change the globle variable pose_prev for next time's use of the function pass-line in the function cls_pt
            pose_prev = pose_curr;
            cout << "Line 421 is good!" << endl;
        
            //Second, update the information of pose_curr by receiving the message 
            pose_curr.x = msg.pose.pose.position.x;
            pose_curr.y = msg.pose.pose.position.y;
            cout << "Line 425 is good!" << endl;
            // //PS: zhenghe: should pose_curr.theta be defined as msg.pose.pose.orientation.z here? No, it has been modified in m.getRPY() 
            // //passing by reference//. PS: m is m(q), created with quaternion.
            geometry_msgs::Quaternion quad_qtn;
            double Roll_mocap, Pitch_mocap;
            quad_qtn.x = msg.pose.pose.orientation.x;
            quad_qtn.y = msg.pose.pose.orientation.y;
            quad_qtn.z = msg.pose.pose.orientation.z;
            quad_qtn.w = msg.pose.pose.orientation.w;
            tf::Quaternion q(quad_qtn.x, quad_qtn.y, quad_qtn.z, quad_qtn.w);
            tf::Matrix3x3 m(q);//quaternion
            m.getRPY(Roll_mocap, Pitch_mocap, pose_curr.theta);
            //pose_curr.theta = msg.theta;

            while (pose_curr.theta > PI)
            pose_curr.theta -= 2*PI;
            while (pose_curr.theta < -PI)
            pose_curr.theta += 2*PI;
            cout << "Line 437 is good!" << endl;

            //Third, find out the closest point to the current the pose
            //PS: pose_curr is 3x1, but cls_pt is 6x1 vector 
            find_closest_pt(pose_curr);
            cout << "Line 486 is good!" << endl;  
            //moreover, well define the pose-des
            pose_des.x = cls_pt[0];
            pose_des.y = cls_pt[1];
            pose_des.theta = cls_pt[2];
            while (pose_des.theta > PI)
            pose_des.theta -= 2*PI;
            while (pose_des.theta < -PI)
            pose_des.theta += 2*PI;

            //Output current conditions
            cout << "CurrKine: vel_lin: "<<cls_pt[6]<<", vel_ang: "<<cls_pt[7]<<", NO.Traj: "<<cls_pt[4]<<endl;
            cout << "CurrDest: x = ("<<pose_des.x<<"), y = ("<<pose_des.y<<"), theta = ("<<pose_des.theta << ")."  << endl
            <<"CurrPose: x = (" << pose_curr.x << "), y = (" << pose_curr.y << "), theta = (" << pose_curr.theta << ")."  << endl;

        
            //Fourth, if-statement for judging whether Jackal goes too far away to the stardard route
            if (distance3(pose_curr, pose_des) > 0.5)
            {
                //always stop the Jackal
                cout << "Bad following, stop forever!" << endl;
                vel_cmd.linear.x = 0;
                vel_cmd.angular.z = 0;
            }        
            //Five, the tracking results is still not too bad, try to clarify the situation
            else
            {
                //if-statement for switching the controller's corresponding trajectory
                if ((it_traj) != (mtraj.end()-1))
                {
                    //i. try to clarify the situation to change the switch first
                    if (pass_line(mpath[_count_+1], pose_curr, pose_prev))
                    {
                        _count_++;
                        it_traj += pt_num;
                        cout << "Line 517 is good!" << endl;
                        if (it_traj == (mtraj.end()-1))
                        {
                            //reach the final cross-line
                            cout << "Right pass the final crossline, we are landing!" << endl;
                        }
                    }
            
                    //ii. use the controller
                    ugv_kinematic_controller();
                    cout << "Line 518 is good!" << endl;
                    
                    ROS_INFO_STREAM("send velocity command:"
                    << " linear=" << vel_cmd.linear.x
                    << " angular=" << vel_cmd.angular.z);
                    cout << "Line 487 is good!" << endl;
                    _pub_control_.publish(vel_cmd); 
                }
            
                //when it passes the last way-point of the whole trajectory
                if (it_traj == (mtraj.end()-1))
                {
                    //always stop the Jackal
                    cout << "Trajectory completed!" << endl;
                    vel_cmd.linear.x = 0;
                    vel_cmd.angular.z = 0;
                }
            }
        }

        //Last, if-statement for judging whether Jackal reaches the stardard destination 
        //when coming to the end
        else if (distance3(pose_curr, pose_des) > 0.5)
        {
            //always stop the Jackal
            cout << "Bad reaching the destination, stop forever!" << endl;
            vel_cmd.linear.x = 0;
            vel_cmd.angular.z = 0;
        }    
    }    

} //namespace jackal_zhenghe