cmake_minimum_required(VERSION 2.8.3)
project(tuw_aruco)

# Set ArUco library path. (The containing cmake file will be added as subproject/library to this project)
set("ARUCO_LIB_DIR" "aruco-3.1.12/")

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

# Build aruco library with CMake
add_subdirectory(${ARUCO_LIB_DIR})

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
        cfg/ArUco.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS cv_bridge image_geometry image_transport image_view opencv tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include/ ${catkin_INCLUDE_DIRS} ${ARUCO_LIB_DIR}src/)

add_executable(aruco_node src/aruco_node.cpp src/aruco_base.cpp src/aruco_parameters.cpp src/aruco_marker_pose.cpp)

## Add cmake target dependencies of the executable
add_dependencies(aruco_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(aruco_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    aruco
)


#############
## Install ##
#############

install(TARGETS aruco_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
