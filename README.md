# Jackal_UGV_PathGeneration_and_Control
**Authors: Jackal UGV Refined Path-Generation and Control** Zhenghe Shangguan


# 1. Main Contribution
- Input: Randomly-distributed way-point with (x,y,theta) information, where (x,y) are the coordinates on the plane, theta is the linear velocity direction.
- Output: A refined and dense-and-uniformly-distributed path with kinematic information on each way-point, a series of suitable control commands while Jackal Robot is running in Gazebo or real world.
## Features:
- Path-adaptive control command: Generate suitable linear and angular velocity control signal according to the curvature radius and the future path;
- Self-awareness of Motion mode: According to the given sparse way-points, Jackal can judge the forward / backward motion mode and generate corresponding control signals;
- Gentle Start and Stop periods: Gradual acceleration and deceleration when Jackal Robot starts or stops.
- Safety: Linear and Augular velocity limitation can be easily set.


# 2. Methodology
## Refined Path Generation:
- Use interpolation to provide continuous-changing velocities.
## Nonlinear Jackal Controller:
Ref: Kanayama et al., “A stable tracking control method for an autonomous mobile robot”, ICRA1990.

# 3. Prerequisites
I have tested the library in **14.04** with ROS indigo.

## C++11 or C++0x Compiler
I use the new thread and chrono functionalities of C++11.

## ROS 
ROS indigo is required [ros](http://wiki.ros.org/indigo/Installation/Ubuntu).

## Jackal Simulation Packages:
Refer to: http://www.clearpathrobotics.com/assets/guides/jackal/simulation.html

## Gazebo Simulation Environment
Refer to: http://gazebosim.org/tutorials?tut=ros_installing

## ROS Packages
- nav_msgs/Odometry: Publishing Odometry Information over ROS (Ref: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)


# 4. Building and Running the Project
## Total Four Terminals needed for this project

## Terminal 1:
Clone the repository:
```
git clone https://github.com/ZhengheShangguan/Jackal_UGV_PathGeneration_and_Control.git
```

```
cd Jackal_UGV_PathGeneration_and_Control
catkin_make
source ./devel/setup.sh
rosrun jackal_zhenghe jackal_zhenghe _param:=./path_examples/taiji.txt
```

## Terminal 2:
```
roscore
```

## Terminal 3:
```
cd odometry
catkin_make
source ./devel/setup.sh
rosrun process_odometry process_odometry
```

## Terminal 4:
```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```


# 5. Run in Vicon Room
```
(This part received some help from UIUC PhD.Xinke Deng (github.com/XinkeAE))
```

If you want to run this path_generator and controller in Vicon room in real world, you should use Vicon's topic to provide Jackal UGV with its pose in real time.
At least you need to make the following changes (Contents may differ according to different motion capture systems):

- Change 1: in function "void Jackal_controller::poseMessageRecieved_odom(const nav_msgs::Odometry & msg)" of "Jackal_controller.cpp"
1. nav_msgs::Odometry & msg -> geometry_msgs::PoseStamped & msg ;
2. Since we changed the msg class, .pose.pose -> .pose ;(6 places total)

- Change 2: in function "int main(int argc, char* argv[])" of "main_gazebo.cpp"
Change the Subscriber: /odometry/relative -> /vrpn_client_node/jackal/pose ;

# 6. Some DEMOs

