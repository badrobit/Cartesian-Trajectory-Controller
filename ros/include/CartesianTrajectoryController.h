/**
 * CartesianTrajectoryController.h
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

#ifndef CARTESIANTRAJECTORYCONTROLLER_H_
#define CARTESIANTRAJECTORYCONTROLLER_H_

#include <ros/ros.h>

// hbrs_srvs/srv/ComputeTrajectory.srv
#include <hbrs_srvs/ComputeTrajectory.h>
// hbrs_srvs/srv/ComputeTrajectory.srv
#include <hbrs_srvs/ExecuteTrajectory.h>

class CartesianTrajectoryController
{
public:

	/**
	 * The constructor for the CartesianTrajectoryController class. It takes in a node handler from
	 * the ros_node which is calling it.
	 */
	CartesianTrajectoryController( ros::NodeHandle i_node_handle );

	/**
	 * Default destructor will be used to free any memory or close any processes that should not be
	 * left around once the program has completed.
	 */
	virtual ~CartesianTrajectoryController();

private:

	/**
	 * This function takes in a list of x,y,z positions which represent points that will be the
	 * positions in 3D coordonates that the arm needs to move to.
	 */
	void ComputeTrajectory( hbrs_srvs::ComputeTrajectory::Request &req,
							hbrs_srvs::ComputeTrajectory::Response &res );

	/**
	 * This function takes the completed trajectory plan that is created by calling the
	 * ComputeTrajectory service and it attempts to execute the trajectory. It watches the progress
	 * of the arm and will inform the caller of the service when the trajectory has been
	 * successfully completed or if there was a problem executing the trajectory
	 * (and what the error is).
	 *
	 * For a list of the possible errors please see the CartesianTrajectoryController.msg
	 */
	void ExecuteTrajectory( hbrs_srvs::ExecuteTrajectory::Request &req,
							hbrs_srvs::ExecuteTrajectory::Response &res );

	/**
	 * This is a simpel trajectory calculator that will take in a start and an end point and split
	 * it up in a way where we are able to leverage the youBot Cartesian Controller to move the arm
	 * along the line by splitting up the line into small enough portions that we are able to
	 * smoothly move along the line.
	 */
	void ComputeTrajectorySimple();

	/**
	 * TBD.
	 */
	void ComputeTrajectoryIK();

	/**
	 * This function handles the closing out of any ROS Publisher, Subscribers.
	 */
	void ShutDown();

protected:
	ros::NodeHandle 			m_node_handler;

	ros::ServiceServer 			m_compute_trajectory_service;
	ros::ServiceServer			m_execute_trajectory_service;
};

#endif /* CARTESIANTRAJECTORYCONTROLLER_H_ */
