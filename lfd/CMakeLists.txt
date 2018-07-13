cmake_minimum_required(VERSION 2.8.3)
project(lfd)

find_package(catkin REQUIRED COMPONENTS
  centroid_detector_msgs
  rospy
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

catkin_python_setup()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add folders to be run by python nosetests
catkin_add_nosetests(tests)
