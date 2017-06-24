
#include <nautonomous_mission_coordinator/move_base_client.h>

MoveBaseClient::MoveBaseClient() 
{
	moveBaseActionClient = new MoveBaseActionClient("move_base", true);
}

MoveBaseClient::~MoveBaseClient() 
{
	if(moveBaseActionClient)
	{
		delete moveBaseActionClient;
	}
}

bool MoveBaseClient::requestGoal(const geometry_msgs::Pose2D pose2d) 
{
	while (!moveBaseActionClient->waitForServer(ros::Duration(5.0))) 
	{
		ROS_INFO("Waiting for the move_base action server to come up");
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

	moveBaseActionClient->sendGoal(goal);

	moveBaseActionClient->waitForResult();

	if (moveBaseActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, succeeded with moving to the next goal.");
		return true;
	}
	else
	{
		ROS_INFO("The base failed to move to the next goal.");
		return false;
	}
	
}
