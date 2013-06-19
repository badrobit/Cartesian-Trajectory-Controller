HBRS Cartesian Trajectory Controller
====================================

This ROS node is designed to take a series of points along a trajectory that you want your robot arm to be able to follow. It will compute a path from the current position of the arm to the desired goal and attempt to follow it as closely as possible. 

### Available Services
* `/compute_trajectory`

## Launching It Simulation:
* Start the Simulation: `$ roslaunch raw_bringup_sim robot.launch`
* Start the Dashboard:  `$ roslaunch raw_bringup dashboard.launch` 
* Start the Trajectory Controller: `$ roslaunch hbrs_cartesian_trajectory_controller cartesian_trajectory_controller_offline.launch`
* Start the Visualization: `$ rosrun rviz rviz`

## Launching It Real Hardware:
* Start the Robot: `$ roslaunch raw_bringup robot.launch`
* Start the Dashboard:  `$ roslaunch raw_bringup dashboard.launch` 
* Start the Trajectory Controller: `$ roslaunch hbrs_cartesian_trajectory_controller cartesian_trajectory_controller_offline.launch`
* Start the Visualization: `$ rosrun rviz rviz`

## Running It: 
In order to start the HBRS Cartesian Trajectory Controller you need to call the service which is provides in the following way from the terminal: 

`$ rosservice call /compute_trajectory [waypoints]`

### WayPoint List
The input arguments take in a PoseArray which lists out in order all of the poses that you want to reach using the arm. 

## Environment Variables
This requires the following environment variables to be present in your bashrc file: 
* `export ROBOT=youbot-hbrs1`

## Testing

In order to test the pacakge go into the `/ros/test` directory and run the following command: 

`$ ./test_trajectories.sh`
