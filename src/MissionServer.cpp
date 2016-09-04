#include "../include/nautonomous_operation_action_client/MissionServer.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nautonomous_operation_action_client/MissionAction.h>


int missionIndex = 0;
const int maxMission = 4;

//Create mission points for the journey in WPK
double missionCoordinates[maxMission][2][4] = {

        { { -55, 38, 0.0, 0.0 }, { 0.0, 0.0, 0.477, 0.879 } },

        { { -45, 53, 0.0, 0.0 }, { 0.0, 0.0, -0.246, 0.969 } },

        { { -31, 46, 0.0, 0.0 }, { 0.0, 0.0, 0.867, -0.499 } },

        { { -36, 18, 0.0, 0.0 }, { 0.0, 0.0, 0.946, 0.325 } } };

MissionServer::MissionServer(ros::NodeHandle nh_, std::string name) :
		as_(nh_, name.c_str(), boost::bind(&MissionServer::executeCB, this, _1),
				false), action_name_(name) {
//	ROS_INFO("Started action %s", action_name.c_str());
	as_.start();
}

MissionServer::~MissionServer(void) {
}

void MissionServer::executeCB(
		const nautonomous_operation_action_client::MissionGoalConstPtr &goal) {
	// helper variables

	goal_.operation = goal->operation;
	nextPosition_ = geometry_msgs::Point(goal_.operation.path[0]);
	nextOrientation_ = geometry_msgs::Quaternion(
			goal_.operation.orientations[0]);

	ros::Rate r(1);
	bool success = true;

	// push_back the seeds for the mission status
	feedback_.feedback.progression = 50; //arbitrary value for now TODO
	feedback_.feedback.status = "Ok";

	// publish info to the console for the user
	ROS_INFO("received goal");

	as_.publishFeedback(feedback_);

	r.sleep();

	if (success) {
		result_.result.progression = 100;
		result_.result.status = feedback_.feedback.status;

		// set the action state to succeeded
		as_.setSucceeded(result_);
	}
}

void MissionServer::getNextGoal(/*tf::TransformListener* listener*/) {
	
	//Select next position, based on the mission coordinate and mission index

	nextPosition_ = geometry_msgs::Point();
	nextPosition_.x = missionCoordinates[missionIndex][0][0];
	nextPosition_.y = missionCoordinates[missionIndex][0][1];
	nextPosition_.z = missionCoordinates[missionIndex][0][2];
	ROS_INFO("Next point is %d (%f, %f, %f)", missionIndex, nextPosition_.x, nextPosition_.y,
			nextPosition_.z);

	nextOrientation_ = geometry_msgs::Quaternion();
	nextOrientation_.x = missionCoordinates[missionIndex][1][0];
	nextOrientation_.y = missionCoordinates[missionIndex][1][1];
	nextOrientation_.z = missionCoordinates[missionIndex][1][2];
	nextOrientation_.w = missionCoordinates[missionIndex][1][3];
	ROS_INFO("Next orientation is (%f, %f, %f, %f)", nextOrientation_.x,
			nextOrientation_.y, nextOrientation_.z, nextOrientation_.w);


	//mission index, start at 0 and end at (maxMission-1)
	missionIndex = (missionIndex + 1) % maxMission;

}

