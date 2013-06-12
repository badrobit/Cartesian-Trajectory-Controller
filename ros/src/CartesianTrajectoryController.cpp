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
	m_compute_trajectory_service =
			m_node_handler.advertiseService( "compute_trajectory", &CartesianTrajectoryController::ComputeTrajectory, this );

	m_sub_joint_states = m_node_handler.subscribe( "/joint_states", 1, &CartesianTrajectoryController::JointStateCallback, this );
	ROS_DEBUG( "Subscribed to the Joint States publication." );

	m_youbot_arm_velocity_publisher =
			m_node_handler.advertise<geometry_msgs::Twist>( "/arm_controller/cartesian_velocity_command", 1 );
}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
}

bool
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
		ComputeTrajectorySimple(req, res);
	}

	return false;
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
CartesianTrajectoryController::ComputeTrajectorySimple( hbrs_srvs::ComputeTrajectory::Request &req,
												  		hbrs_srvs::ComputeTrajectory::Response &res )
{
	ROS_WARN( "Starting simplified solver for trajectory calculations" );
	//Get all future path nodes
	//geometry_msgs::PoseStamped next_pose = req.way_point_list.pose[0];	//Compute Direction Vector

	//m_direction_vec.twist.linear.x = (next_pose.pose.position.x - m_current_gripper_pose.pose.position.x);
	//m_direction_vec.twist.linear.y = (next_pose.pose.position.y - m_current_gripper_pose.pose.position.y);
	//m_direction_vec.twist.linear.z = (next_pose.pose.position.z - m_current_gripper_pose.pose.position.z);

	m_direction_vec.linear.x = 5;
	m_direction_vec.linear.y = 0;
	m_direction_vec.linear.z = 0;

	m_direction_vec.angular.x = 0;
	m_direction_vec.angular.y = 0;
	m_direction_vec.angular.z = 0;

	m_youbot_arm_velocity_publisher.publish( m_direction_vec );

	res.output_value.output_code = hbrs_msgs::CartesianTrajectoryController::SUCCESS;

}

void
CartesianTrajectoryController::ComputeTrajectoryIK()
{

}

void
CartesianTrajectoryController::JointStateCallback( sensor_msgs::JointStateConstPtr joints )
{
	for (unsigned i = 0; i < joints->position.size(); i++)
	{
		ROS_DEBUG_STREAM( "Joint Name: " << joints->name[i].c_str() );
		ROS_DEBUG_STREAM( "Updated Gripper Position: " << joints->position[i] );
	}
}
