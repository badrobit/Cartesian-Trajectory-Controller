HBRS Cartesian Trajectory Controller
====================================

This ROS node is designed to take a series of points along a trajectory that you want your robot arm to be able to follow. It will compute a path from the current position of the arm to the desired goal and attempt to follow it as closely as possible. 

### Available Services
* `/compute_trajectory

## Launching It
* `roslaunch raw_bringup_sim robot.launch`
* `roslaunch hbrs_cartesian_trajectory_controller cartesian_trajectory_controller_offline.launch`
* `rosrun rviz rviz`

## Running It: 
In order to start the HBRS Cartesian Trajectory Controller you need to call the service which is provides in the following way from the terminal: 

`rosservice call /compute_trajectory "use_ik_solution: false" < waypointlist.txt`

### WayPoint List
The input artuments to the 

## Environment Variables
This requires the following environment variables to be present in your bashrc file: 
* `export ROBOT=youbot-hbrs1`
