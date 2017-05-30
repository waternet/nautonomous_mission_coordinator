
#include <../include/nautonomous_operation_action/move_base_action_client.h>

MoveBaseActionClient::MoveBaseActionClient() {
	ac = new MoveBaseClient("move_base", true);
}

MoveBaseActionClient::~MoveBaseActionClient() {
	if(ac){
		delete ac;
	}
}

int MoveBaseActionClient::cancelGoal(){

	if(ac){
		ac->cancelAllGoals();
		return 1;
	}
	return 0;	
}


int MoveBaseActionClient::requestGoal(geometry_msgs::Point point, geometry_msgs::Quaternion quaternion) {

	ROS_INFO("Requesting goal");

	while (!ac->waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal goal;
	
	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "odom_combined";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Client publish the wanted position of the robot
	goal.target_pose.pose.position.x = point.x;
	goal.target_pose.pose.position.y = point.y;
	goal.target_pose.pose.position.z = point.z;
	
	//Client publish the wanted orientation of the robot
	goal.target_pose.pose.orientation.x = quaternion.x;
	goal.target_pose.pose.orientation.y = quaternion.y;
	goal.target_pose.pose.orientation.z = quaternion.z;
	goal.target_pose.pose.orientation.w = quaternion.w;

	ROS_INFO("Sending goal ");
	ac->sendGoal(goal);

	ac->waitForResult();

	if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the base moved 1 meter forward");
	else
		ROS_INFO("The base failed to move forward 1 meter for some reason");

	return 0;
}
