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
	m_sub_joint_states = m_node_handler.subscribe( "/joint_states", 1, &CartesianTrajectoryController::JointStateCallback, this );
	ROS_INFO( "Subscribed to the Joint States publication." );

	m_youbot_arm_velocity_publisher = m_node_handler.advertise<geometry_msgs::TwistStamped>("/hbrs_manipulation/arm_cart_control/cartesian_velocity_command", 1 );
	ROS_INFO( "Started publishing Arm Velocity Commands" );

	m_compute_trajectory_service = m_node_handler.advertiseService( "compute_trajectory", &CartesianTrajectoryController::ComputeTrajectory, this );
	ROS_INFO( "Advertised Compute Trajectory Server." );

	SetupYoubotArm();
}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
}

bool
CartesianTrajectoryController::ComputeTrajectory( hbrs_srvs::ComputeTrajectory::Request &req,
												  hbrs_srvs::ComputeTrajectory::Response &res )
{
	bool return_value = false;

	if( req.use_ik_solver )
	{
		ROS_WARN( "Using inverse kinematics for trajectory calculations" );
		return_value = ComputeTrajectoryIK();
	}
	else
	{
		ROS_WARN( "Using simplified solver for trajectory calculations" );
		return_value = ComputeTrajectorySimple(req, res);
	}

	return return_value;
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

bool
CartesianTrajectoryController::ComputeTrajectorySimple( hbrs_srvs::ComputeTrajectory::Request &req,
												  		hbrs_srvs::ComputeTrajectory::Response &res )
{
	ROS_INFO( "Starting simplified solver for trajectory calculations" );

	ROS_ASSERT( m_arm_joint_names.size() != 0 );

	m_arm_velocities.header.frame_id = "/base_link";
	m_arm_velocities.twist.linear.x = 0;
	m_arm_velocities.twist.linear.y = 1;
	m_arm_velocities.twist.linear.z = 0;

	ros::Time begin = ros::Time::now();
	double duration = 0;
	while( duration < 10 )
	{
		m_youbot_arm_velocity_publisher.publish( m_arm_velocities );
		duration = ros::Time::now().toSec() - begin.toSec();
	}

	res.output_value.output_code = hbrs_msgs::CartesianTrajectoryController::SUCCESS;
	return true;
}

bool
CartesianTrajectoryController::ComputeTrajectoryIK()
{
	return false;
}

void
CartesianTrajectoryController::JointStateCallback( sensor_msgs::JointStateConstPtr joints )
{
	for (unsigned i = 0; i < joints->position.size(); i++)
	{
		//ROS_WARN_STREAM( "Joint Name: " << joints->name[i].c_str() << " Updated Position: " << joints->position[i] );
	}
}

void
CartesianTrajectoryController::SetupYoubotArm()
{
	XmlRpc::XmlRpcValue parameter_list;
	if( m_node_handler.getParam("/arm_controller/joints", parameter_list) )
	{
		ROS_ASSERT(parameter_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < parameter_list.size(); ++i)
		{
			ROS_ASSERT(parameter_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			m_arm_joint_names.push_back(static_cast<std::string>(parameter_list[i]));
		}

		ROS_WARN_STREAM( "Found " << m_arm_joint_names.size() << " joints for the youBot arm." );

		//read joint limits
		for(unsigned int i=0; i < m_arm_joint_names.size(); ++i)
		{
			arm_navigation_msgs::JointLimits joint_limits;
			joint_limits.joint_name = m_arm_joint_names[i];
			m_node_handler.getParam("/arm_controller/limits/" + m_arm_joint_names[i] + "/min", joint_limits.min_position);
			m_node_handler.getParam("/arm_controller/limits/" + m_arm_joint_names[i] + "/max", joint_limits.max_position);
			m_upper_joint_limits.push_back( joint_limits.max_position );
			m_lower_joint_limits.push_back( joint_limits.min_position );
		}

		//m_arm_velocities.velocities.clear();

		ROS_INFO( "youBot Arm has been initialized." );
	}
	else
	{
		ROS_ERROR( "youBot Arm Failed to initialize" );
	}
}
