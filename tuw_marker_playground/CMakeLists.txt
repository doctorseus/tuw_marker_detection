cmake_minimum_required(VERSION 2.8.3)
project(tuw_marker_playground)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

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

## System dependencies
find_package( OpenCV REQUIRED )

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
        cfg/Dice.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    DEPENDS OpenCV
)
###########
## Build ##
###########

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

add_executable(mpg_node
    src/mpg_node.cpp
)

add_executable(mpg_ellipses_node
    src/mpg_ellipses_node.cpp
)

add_executable(mpg_dice_node
    src/mpg_dice_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(mpg_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(mpg_ellipses_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(mpg_dice_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)