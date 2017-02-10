cmake_minimum_required(VERSION 2.8.3)
project(tuw_marker_benchmark)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    image_geometry
    image_transport
    tf
    cv_bridge
    marker_msgs
    dynamic_reconfigure
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    DEPENDS dynamic_reconfigure roscpp rospy std_msgs
)

