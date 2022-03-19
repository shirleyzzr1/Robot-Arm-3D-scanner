cmake_minimum_required(VERSION 3.0.2)
project(realsense_reader)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation sensor_msgs tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros roscpp rospy 
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  PointCloudArray.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  savepc.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  sensor_msgs# Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES realsense_reader
  CATKIN_DEPENDS message_runtime sensor_msgs tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros roscpp rospy 
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/read_camera.py
  nodes/publish_armstate.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(pcl_process nodes/pcl_process.cpp)
target_link_libraries(pcl_process ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(pcl_new nodes/pcl_new.cpp)
target_link_libraries(pcl_new ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_realsense_reader.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
