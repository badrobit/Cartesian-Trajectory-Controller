/**
 * cartesian_trajectory_controller.cpp
 *
 * Copyright (c) May 28, 2013, Matthew Roscoe, Adam Gaier
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <CartesianTrajectoryController.h>
#include <ros/console.h>

/**
 * 	This is the start point for the hbrs_cartesian_trajctory_controller software. In order to start
 * 	the software you need to run the following command:
 *
 * 	roslaunch hbrs_cartesian_trajectory_controller online.launch
 */
int main(int argc, char **argv)
{
	/*
	 * Start up ROS and create our node handler.
	 */
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle node_handler;

	/*
	 * Create our CartesianTrajectoryController object and give it our node_handler so that it is
	 * able to communicate with the ROS master and other nodes which are running.
	 */
	CartesianTrajectoryController ctc( node_handler );
	ros::spin();

	/*
	 * Default close out for a C/C++ program.
	 */
	return 0;
}
