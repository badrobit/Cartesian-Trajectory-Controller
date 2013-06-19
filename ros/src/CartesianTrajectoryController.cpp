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

	m_ctc_goal_marker_publisher = m_node_handler.advertise<visualization_msgs::Marker>( "ctc_goal_marker", 0 );

	SetupYoubotArm();
}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
}

bool
CartesianTrajectoryController::ComputeTrajectory( hbrs_srvs::ComputeTrajectory::Request &req,
												  hbrs_srvs::ComputeTrajectory::Response &res )
{
	ROS_ASSERT( req.way_point_list.poses.size() != 0 );

	for( int i = 0; i < req.way_point_list.poses.size(); i ++ )
	{
		bool done_x_movement = false;
		bool done_y_movement = false;
		bool done_z_movement = false;

		ROS_ASSERT( m_arm_joint_names.size() != 0 );

		geometry_msgs::PoseStamped next_pose;
		next_pose.pose.position.x = req.way_point_list.poses[i].position.x;
		next_pose.pose.position.y = req.way_point_list.poses[i].position.y;
		next_pose.pose.position.z = req.way_point_list.poses[i].position.z;

		ROS_INFO_STREAM( "Starting movement to WayPoint #" << i << " [" << next_pose.pose.position.x <<
				", " << next_pose.pose.position.y << ", " << next_pose.pose.position.z << "] " );

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time();
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = next_pose.pose.position.x;
		marker.pose.position.y = next_pose.pose.position.y;
		marker.pose.position.z = next_pose.pose.position.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		m_ctc_goal_marker_publisher.publish( marker );

		while(  !( done_x_movement && done_y_movement && done_z_movement )  )
		{
			UpdateGripperPosition();

			double x_difference = m_current_gripper_pose.pose.position.x - next_pose.pose.position.x;
			double y_difference = m_current_gripper_pose.pose.position.y - next_pose.pose.position.y;
			double z_difference = m_current_gripper_pose.pose.position.z - next_pose.pose.position.z;

			//ROS_WARN_STREAM( "Difference = [ " << x_difference << " , " << y_difference << " , " << z_difference << " ]" );
			//ROS_WARN_STREAM( "Done in x: " << done_x_movement << " y: " << done_y_movement << " z: " << done_z_movement );

			double normalized_velocities = fabs( x_difference ) + fabs( y_difference ) + fabs( z_difference );

			m_arm_velocities.header.frame_id = "/base_link";


			if( x_difference >= m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.x = -( fabs(x_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_x_movement = false;
			}
			else if( x_difference <= -m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.x = ( fabs(x_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_x_movement = false;
			}
			else
			{
				ROS_WARN( "STOPPING X" );
				m_arm_velocities.twist.linear.x = 0;
				done_x_movement = true;
			}


			if( y_difference >= m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.y = -( fabs(y_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_y_movement = false;
			}
			else if( y_difference <= -m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.y = ( fabs(y_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_y_movement = false;
			}
			else
			{
				ROS_WARN( "STOPPING Y" );
				m_arm_velocities.twist.linear.y = 0;
				done_y_movement = true;
			}


			if( z_difference >= m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.z = -( fabs(z_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_z_movement = false;
			}
			else if( z_difference <= -m_arm_position_tolerance )
			{
				m_arm_velocities.twist.linear.z = ( fabs(z_difference) / normalized_velocities ) * m_arm_velocity_rate;
				done_z_movement = false;
			}
			else
			{
				ROS_WARN( "STOPPING Z" );
				m_arm_velocities.twist.linear.z = 0;
				done_z_movement = true;
			}

			m_youbot_arm_velocity_publisher.publish( m_arm_velocities );
			ros::Duration(0.05).sleep();
		}

		ROS_INFO_STREAM( "Finished with WayPoint #" << i << " moving to next WayPoint" );
	}

	ROS_INFO_STREAM( "Finished following provided Trajectory" );
	res.output_value.output_code = hbrs_msgs::CartesianTrajectoryController::SUCCESS;
	return true;
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
CartesianTrajectoryController::JointStateCallback( sensor_msgs::JointStateConstPtr joints )
{
	for (unsigned i = 0; i < joints->position.size(); i++)
	{
		//UpdateGripperPosition();
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

bool
CartesianTrajectoryController::UpdateGripperPosition()
{
	ROS_DEBUG( "Updating current youbot position" );

	tf::StampedTransform transform;
	try
	{
		m_transform_listener.lookupTransform( "/base_link", "/arm_link_5", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

	m_current_gripper_pose.pose.position.x = transform.getOrigin().x();
	m_current_gripper_pose.pose.position.y = transform.getOrigin().y();
	m_current_gripper_pose.pose.position.z = transform.getOrigin().z();

	ROS_DEBUG_STREAM( "Updated Grpper Position ( " << m_current_gripper_pose.pose.position.x << ", " <<
			m_current_gripper_pose.pose.position.y << ", " << m_current_gripper_pose.pose.position.z << " )" );

	return true;

}
