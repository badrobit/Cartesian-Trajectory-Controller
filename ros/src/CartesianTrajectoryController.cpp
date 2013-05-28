/**
 * CartesianTrajectoryController.cpp
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

#include "CartesianTrajectoryController.h"

CartesianTrajectoryController::CartesianTrajectoryController( ros::NodeHandle i_node_handle ): m_node_handler( i_node_handle )
{
	//m_compute_trajectory_service =
	//		m_node_handler.advertiseService( "compute_trajectory", &CartesianTrajectoryController::ComputeTrajectory, this );
	ROS_INFO( "Advertised 'do_visual_servoing' service for raw_visual_servoing" );
}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
}

void
CartesianTrajectoryController::ComputeTrajectory( hbrs_srvs::ComputeTrajectory::Request &req,
												  hbrs_srvs::ComputeTrajectory::Response &res )
{
	if( req.use_ik_solver )
	{
		ROS_WARN( "Using inverse kinematics for trajectory calculations" );
		ComputeTrajectoryIK();
	}
	else
	{
		ROS_WARN( "Using simplified solver for trajectory calculations" );
		ComputeTrajectorySimple();
	}
}

void
CartesianTrajectoryController::ExecuteTrajectory( hbrs_srvs::ExecuteTrajectory::Request &req,
												  hbrs_srvs::ExecuteTrajectory::Response &res )
{

}

void
CartesianTrajectoryController::ShutDown()
{

}

void
CartesianTrajectoryController::ComputeTrajectorySimple()
{

}

void
CartesianTrajectoryController::ComputeTrajectoryIK()
{

}
