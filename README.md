HBRS Cartesian Trajectory Controller
====================================

This ROS node is designed to take a series of points along a trajectory that you want your robot arm to be able to follow. It will compute a path from the current position of the arm to the desired goal and attempt to follow it as closely as possible. 

### Available Services
* `/compute_trajectory`

## Launching It Simulation:

This will bring up everything you need in order to set up the robot in simulation and to move and track its output. This will also launch RVIZ which will visualize the points and both the ideal trajectory and the actual trajectory that the robot ended up following while executing the trajectory. 

* Start the Simulation: `$ roslaunch raw_bringup_sim robot.launch`
* Start the Dashboard:  `$ roslaunch raw_bringup dashboard.launch` 
* Start the Trajectory Controller: `$ roslaunch hbrs_cartesian_trajectory_controller cartesian_trajectory_controller_offline.launch`

## Launching It Real Hardware:

This will launch the actual youbot and initialize it and launch the componenet from a remote machine (if you do this the youbot needs to be set to your `ROS_MASTER`). It will also launch RVIZ which will allow you to visualize the output from the robot with a slight delay depending on your setup. 

* Start the Robot: `$ roslaunch raw_bringup robot.launch`
* Start the Dashboard:  `$ roslaunch raw_bringup dashboard.launch` 
* Start the Trajectory Controller: `$ roslaunch hbrs_cartesian_trajectory_controller cartesian_trajectory_controller_offline.launch`

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
