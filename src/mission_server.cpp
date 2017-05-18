#include "../include/nautonomous_operation_action/mission_server.h"
#include <gps_common/conversions.h>

int missionIndex = 0;

/**
 * Create mission points for the journey in WPK
 * coördinate (X, Y, Z)
 * orientatie (X, Y, Z, W) W = 1 en Z = 0 orientatie langs x W = 0 en Z = 0 orientatie langs -x
 * illustratie http://quaternions.online/
 */



//Coenhaven
double missionCoordinatesGPS_coenhaven[4][2] = {
	{52.404369, 4.863451},
	{52.404641, 4.863870},
	{52.404710, 4.863681},
	{52.404429, 4.863370}
};


//Brug hoofdkantoor
double missionCoordinatesGPS[2][2] = {
	{52.341375, 4.917760},
	{52.341450, 4.917626}
};

/*
//Lijnbaansgracht
double missionCoordinatesGPS[2][2] = {
	{52.359991, 4.896177},
	{52.359739, 4.895603}
};
*/

/**
 *\brief Constructor for MissionServer
 *\param ros::NodeHandle nh_
 *\param string name
 *\return MissionServer
 */
MissionServer::MissionServer(ros::NodeHandle nh_, std::string name) :
		as_(nh_, name.c_str(), boost::bind(&MissionServer::executeCB, this, _1),
				false), action_name_(name) {
//	ROS_INFO("Started action %s", action_name.c_str());
	as_.start();
}

/**
 *\brief Empty constructor for MissionServer
 *\param 
 *\return MissionServer
 */
MissionServer::~MissionServer(void) {
}

/**
 *\brief Empty constructor for MissionServer
 *\param nautonomous_operation_action::MissionGoalConstPtr &goal
 *\return
 */
void MissionServer::executeCB(const nautonomous_operation_action::MissionGoalConstPtr &goal) {
	
	ROS_INFO("Mission index: %d", missionIndex);

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

/**
 *\brief 
 */
void MissionServer::getNextGoal(/*tf::TransformListener* listener*/ bool simulate) {

	ROS_INFO("Getting next goal");

	ros::NodeHandle node;

 	double map_utm_x, map_utm_y, goal_utm_x, goal_utm_y;
	std::string map_zone, goal_zone;

	//Center off map is simulated or set by map cropper
    if(simulate){
        //Coenhaven missions: missionCoordinatesGPS_coenhaven
		ROS_INFO("GPS goal coenhaven");
		gps_common::LLtoUTM(missionCoordinatesGPS_coenhaven[missionIndex][0], missionCoordinatesGPS_coenhaven[missionIndex][1], goal_utm_y, goal_utm_x, goal_zone);
    }else{
        //Other: missionCoordinatesGPS
		gps_common::LLtoUTM(missionCoordinatesGPS[missionIndex][0], missionCoordinatesGPS[missionIndex][1], goal_utm_y, goal_utm_x, goal_zone);
    } 

	gps_common::LLtoUTM(map_latitude, map_longitude, map_utm_y, map_utm_x, map_zone);

	double next_goal_x = goal_utm_x - map_utm_x;
	double next_goal_y = goal_utm_y - map_utm_y;

	nextPosition_ = geometry_msgs::Point();
	nextPosition_.x = next_goal_x;
	nextPosition_.y = next_goal_y;
	nextPosition_.z = 0.0;
	ROS_INFO("Next point is %d (%f, %f, %f)", missionIndex, nextPosition_.x, nextPosition_.y,
			nextPosition_.z);

	nextOrientation_ = geometry_msgs::Quaternion();
	nextOrientation_.x = 0.0;
	nextOrientation_.y = 0.0;
	nextOrientation_.z = 0.0;
	nextOrientation_.w = 1.0;
	
	/*
	//Select next position, based on the mission coordinate and mission index
	nextPosition_ = geometry_msgs::Point();
	nextPosition_.x = missionCoordinates[missionIndex][0][0];
	nextPosition_.y = missionCoordinates[missionIndex][0][1];
	nextPosition_.z = missionCoordinates[missionIndex][0][2];
	ROS_INFO("Next point is %d (%f, %f, %f)", missionIndex, nextPosition_.x, nextPosition_.y,
			nextPosition_.z)

	nextOrientation_ = geometry_msgs::Quaternion();
	nextOrientation_.x = missionCoordinates[missionIndex][1][0];
	nextOrientation_.y = missionCoordinates[missionIndex][1][1];
	nextOrientation_.z = missionCoordinates[missionIndex][1][2];
	nextOrientation_.w = missionCoordinates[missionIndex][1][3];
	*/

	ROS_INFO("Next orientation is (%f, %f, %f, %f)", nextOrientation_.x,
			nextOrientation_.y, nextOrientation_.z, nextOrientation_.w);
			
	//TODO:
	//int maxMission = sizeof(missionCoordinatesGPS)/sizeof(missionCoordinatesGPS[0]);
	int maxMission = 2;

	//mission index, start at 0 and end at (maxMission-1)
	missionIndex = (missionIndex + 1) % maxMission;
}

