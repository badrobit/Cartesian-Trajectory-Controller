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
	m_youbot_arm_velocity_publisher = m_node_handler.advertise<geometry_msgs::TwistStamped>("/hbrs_manipulation/arm_cart_control/cartesian_velocity_command", 1 );
	ROS_INFO( "Started publishing Arm Velocity Commands" );

	m_compute_trajectory_service = m_node_handler.advertiseService( "compute_trajectory", &CartesianTrajectoryController::ComputeTrajectory, this );
	ROS_INFO( "Advertised Compute Trajectory Server." );

	m_ctc_marker_publisher = m_node_handler.advertise<visualization_msgs::Marker>( "ctc_markers", 10 );
	ROS_INFO( "Started Goal Point and Trajectory marker publication" );

	SetupYoubotArm();
}

CartesianTrajectoryController::~CartesianTrajectoryController()
{
}

bool
CartesianTrajectoryController::ComputeTrajectory( hbrs_srvs::ComputeTrajectory::Request &req,
												  hbrs_srvs::ComputeTrajectory::Response &res )
{
	/**
	 * The following statements are used to catch any conditions under which we should not be performing any of the
	 * computations that follow:
	 *
	 * 1 - The waypoint list size is equal to zero.
	 * 2 - The youbot arm is not initialized and able to be used (determined by there being no parameters available).
	 */
	ROS_ASSERT( req.way_point_list.poses.size() != 0 );
	ROS_ASSERT( m_arm_joint_names.size() != 0 );

	ROS_INFO_STREAM( "Starting Trajectory for " << req.way_point_list.poses.size() << " waypoints" );

	visualization_msgs::Marker goal_points, ideal_trajectory, real_trajectory, mid_points;
	goal_points.header.frame_id = ideal_trajectory.header.frame_id = real_trajectory.header.frame_id = mid_points.header.frame_id = "/base_link";
	goal_points.header.stamp = ideal_trajectory.header.stamp = real_trajectory.header.stamp = mid_points.header.stamp = ros::Time();
	goal_points.ns = ideal_trajectory.ns = real_trajectory.ns = mid_points.ns = "ctc_visual_output";
	goal_points.type = visualization_msgs::Marker::POINTS;
	ideal_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
	real_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
	mid_points.type = visualization_msgs::Marker::LINE_LIST;
	goal_points.action = ideal_trajectory.action = real_trajectory.action = mid_points.action = visualization_msgs::Marker::ADD;
	goal_points.pose.orientation.w = ideal_trajectory.pose.orientation.w = real_trajectory.pose.orientation.w = mid_points.pose.orientation.w = 1.0;

	goal_points.id = 0;
	ideal_trajectory.id = 1;
	real_trajectory.id = 2;
	mid_points.id = 3;

	goal_points.scale.x = m_arm_position_tolerance;
	goal_points.scale.y = m_arm_position_tolerance;
	goal_points.scale.z = m_arm_position_tolerance;

	ideal_trajectory.scale.x = 0.01;
	real_trajectory.scale.x = 0.01;
	mid_points.scale.x = 0.01;

	goal_points.color.a = 1.0;
	goal_points.color.g = 1.0;
	ideal_trajectory.color.a = 1.0;
	ideal_trajectory.color.b = 1.0;
	real_trajectory.color.a = 1.0;
	real_trajectory.color.r = 1.0;
	mid_points.color.a = 1.0;
	mid_points.color.g = 1.0;

	for( int i = 0; i < (int)req.way_point_list.poses.size(); i ++ )
	{
		geometry_msgs::PoseStamped next_pose;
		next_pose.header = req.way_point_list.header;
		next_pose.pose.position.x = req.way_point_list.poses[i].position.x;
		next_pose.pose.position.y = req.way_point_list.poses[i].position.y;
		next_pose.pose.position.z = req.way_point_list.poses[i].position.z;

		ROS_INFO_STREAM( "Starting movement to WayPoint #" << i << " [" << next_pose.pose.position.x <<
							", " << next_pose.pose.position.y << ", " << next_pose.pose.position.z << "] " );

		std::vector<geometry_msgs::PoseStamped> WayPointList = GetWayPoints( m_current_gripper_pose, next_pose );
		ROS_DEBUG_STREAM( "WayPoints: " << WayPointList.size() );

		geometry_msgs::Point p;
		p.x = next_pose.pose.position.x;
		p.y = next_pose.pose.position.y;
		p.z = next_pose.pose.position.z;
		goal_points.points.push_back( p );
		m_ctc_marker_publisher.publish( goal_points );

		for( int x = 0; x < (int)WayPointList.size(); x ++ )
		{
			// reset our movement markers for each subgoal that we need to follow.
			bool done_x_movement = false;
			bool done_y_movement = false;
			bool done_z_movement = false;

			geometry_msgs::PoseStamped sub_goal;
			sub_goal.header = WayPointList[x].header;
			sub_goal.pose.position.x = WayPointList[x].pose.position.x;
			sub_goal.pose.position.y = WayPointList[x].pose.position.y;
			sub_goal.pose.position.z = WayPointList[x].pose.position.z;

			ROS_INFO_STREAM( "Starting movement to sub WayPoint #" << x << " [" << sub_goal.pose.position.x <<
					", " << sub_goal.pose.position.y << ", " << sub_goal.pose.position.z << "] " );

			// Get the subgoal positions turn them into a point and publish them as an RVIZ marker.
			geometry_msgs::Point p1;
			p1.x = sub_goal.pose.position.x;
			p1.y = sub_goal.pose.position.y;
			p1.z = sub_goal.pose.position.z;
			ideal_trajectory.points.push_back( p1 );

			// This type of marker needs two values in x,y,z for the same point so we increase z by 0.3 so that we can
			// see the marker when it is being visualized in RVIZ.
			mid_points.points.push_back( p1 );
			p1.z += 0.3;
			mid_points.points.push_back( p1 );
			m_ctc_marker_publisher.publish( mid_points );
			m_ctc_marker_publisher.publish( ideal_trajectory );

			while(  !( done_x_movement && done_y_movement && done_z_movement )  )
			{
				// In the block below we get the updated position of the gripper and we publish it as a point along a
				// trajectory which we then send to RVIZ. This allows for us to visualize where the arm thinks it is
				// moving and we can plot it in relation to the ideal line that we are supposed to follow.
				UpdateGripperPosition();
				geometry_msgs::Point p2;
				p2.x = m_current_gripper_pose.pose.position.x;
				p2.y = m_current_gripper_pose.pose.position.y;
				p2.z = m_current_gripper_pose.pose.position.z;
				real_trajectory.points.push_back( p2 );
				m_ctc_marker_publisher.publish( real_trajectory );

				// Get the differences between where we are (m_current_gripper_pose) and where we want the gripper to be
				// (sub_goal).
				double x_difference = m_current_gripper_pose.pose.position.x - sub_goal.pose.position.x;
				double y_difference = m_current_gripper_pose.pose.position.y - sub_goal.pose.position.y;
				double z_difference = m_current_gripper_pose.pose.position.z - sub_goal.pose.position.z;
				ROS_DEBUG_STREAM( "Difference = [ " << x_difference << " , " << y_difference << " , " << z_difference << " ]" );
				ROS_DEBUG_STREAM( "Done in x: " << done_x_movement << " y: " << done_y_movement << " z: " << done_z_movement );

				// Normalize the velocities [http://en.wikipedia.org/wiki/Normalization_(statistics)]
				double normalized_velocities = fabs( x_difference ) + fabs( y_difference ) + fabs( z_difference );

				// Set the header value for the output velocities and then compute the actual velcoty that we want to
				// set in each direction. Note this is being done in Cartesian space so we are setting an x,y,z speeds
				// not seperate joint speeds.
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
					ROS_DEBUG( "STOPPING X" );
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
					ROS_DEBUG( "STOPPING Y" );
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
					ROS_DEBUG( "STOPPING Z" );
					m_arm_velocities.twist.linear.z = 0;
					done_z_movement = true;
				}

				m_youbot_arm_velocity_publisher.publish( m_arm_velocities );
				ros::Duration(0.05).sleep();
			}
			ROS_INFO_STREAM( "Finished with SubWayPoint #" << x << " of: " << WayPointList.size() );
		}

		ROS_INFO_STREAM( "Finished with WayPoint #" << i << " moving to next WayPoint" );
	}

	ROS_INFO_STREAM( "Finished following provided Trajectory" );
	res.output_value.output_code = hbrs_msgs::CartesianTrajectoryController::SUCCESS;
	return true;
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
		m_transform_listener.lookupTransform( "/base_link", "/marker_pos", ros::Time(0), transform);
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

std::vector<geometry_msgs::PoseStamped> 
CartesianTrajectoryController::GetWayPoints(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
{
	ROS_ASSERT( p1.header.frame_id != p2.header.frame_id );
	ROS_INFO( "Calculating SubWaypoint Positions" );

	std::vector<geometry_msgs::PoseStamped> way_points;

	float distance = sqrt( pow( (p1.pose.position.x - p2.pose.position.x) ,2) +
                    		pow( (p1.pose.position.y - p2.pose.position.y) ,2) +
                    		pow( (p1.pose.position.z - p2.pose.position.z) ,2) );

	int number_of_way_points = ceil(distance/m_way_point_resolution);
	float interval = 1/number_of_way_points;

	double dx = p1.pose.position.x + (p2.pose.position.x - p1.pose.position.x);
	double dy = p1.pose.position.y + (p2.pose.position.y - p1.pose.position.y);
	double dz = p1.pose.position.z + (p2.pose.position.z - p1.pose.position.z);

	way_points.push_back(p1);
	for (int i=0; i<number_of_way_points; i++)
	{
		geometry_msgs::PoseStamped next_point = way_points.at(i);

		next_point.pose.position.x += dx*interval;
		next_point.pose.position.y += dy*interval;
		next_point.pose.position.z += dz*interval;

		way_points.push_back(next_point);
	}
	way_points.push_back(p2);

	for( int x = 0; x < (int)way_points.size(); x ++ )
	{
		ROS_WARN_STREAM( "SubWayPoint #" << x << " [" << way_points[x].pose.position.x << "," <<
														 way_points[x].pose.position.y << "," <<
														 way_points[x].pose.position.y << "]" );
	}

	return way_points;
}
