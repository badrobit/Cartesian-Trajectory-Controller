cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#..: Default Paths :...............................................................................#
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(
   ${PROJECT_SOURCE_DIR}/ros/include
)

#..: Cartesian Trajectory Controller :.............................................................#
rosbuild_add_library( CartesianTrajectoryController ros/src/CartesianTrajectoryController.cpp )

#..: HBRS Catresian Trajectory Controller ROS node :...............................................#
rosbuild_add_executable( ctc_node ros/nodes/cartesian_trajectory_controller.cpp)
target_link_libraries(ctc_node CartesianTrajectoryController )