cmake_minimum_required(VERSION 2.8.3)
project(jackal_zhenghe)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES template_comm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


## Declare a C++ executable
 add_executable(jackal_zhenghe main_gazebo.cpp Jackal_controller.cpp Traj_generator.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(jackal_zhenghe
   ${catkin_LIBRARIES}
 )

