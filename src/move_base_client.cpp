
#include <nautonomous_mission_coordinator/move_base_client.h>

MoveBaseClient::MoveBaseClient() 
{
	move_base_action_client_ = new MoveBaseActionClient("move_base", true);
}

MoveBaseClient::~MoveBaseClient() 
{
	if(move_base_action_client_)
	{
		delete move_base_action_client_;
	}
}

bool MoveBaseClient::requestGoal(const geometry_msgs::Pose2D pose2d) 
{
	while (!move_base_action_client_->waitForServer(ros::Duration(5.0))) 
	{
		ROS_ERROR("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;
	
	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "odom_combined";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Client publish the wanted position of the robot
	goal.target_pose.pose.position.x = pose2d.x;
	goal.target_pose.pose.position.y = pose2d.y;
	goal.target_pose.pose.position.z = 0;
	
	//Client publish the wanted orientation of the robot
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = std::sin(pose2d.theta * 0.5);
	goal.target_pose.pose.orientation.w = std::cos(pose2d.theta * 0.5);

	ROS_INFO("Sending goal");
	move_base_action_client_->sendGoal(goal);

	move_base_action_client_->waitForResult();

	return (move_base_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
	
}
