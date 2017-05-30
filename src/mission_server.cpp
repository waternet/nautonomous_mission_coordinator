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
	{52.342372, 4.918400},
	{52.342080, 4.917244}
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
*\brief Calculating next goal, if index -1, use current goal
*\param int index
*/


void MissionServer::calculateGoal(int index){
	if(index == -1){
		ROS_INFO("NULL received");
		index = missionIndex;
	}

	ROS_INFO("Simulating: %d", simulate);

 	double map_utm_x, map_utm_y, goal_utm_x, goal_utm_y;
	std::string map_zone, goal_zone;

	ROS_INFO("Used GPS map: %f / %f", map_latitude, map_longitude);

	//Center of map is simulated or set by map cropper
    if(simulate){
        //Coenhaven missions: missionCoordinatesGPS_coenhaven
		gps_common::LLtoUTM(missionCoordinatesGPS_coenhaven[index][0], missionCoordinatesGPS_coenhaven[index][1], goal_utm_y, goal_utm_x, goal_zone);
		ROS_INFO("Used GPS coenhaven goal: %f / %f", missionCoordinatesGPS_coenhaven[index][0], missionCoordinatesGPS_coenhaven[index][1]);
    }else{
        //Other: missionCoordinatesGPS
		gps_common::LLtoUTM(missionCoordinatesGPS[index][0], missionCoordinatesGPS[index][1], goal_utm_y, goal_utm_x, goal_zone);
		ROS_INFO("Used GPS goal: %f / %f", missionCoordinatesGPS[index][0], missionCoordinatesGPS[index][1]);
    } 

	//gps_common::LLtoUTM(missionCoordinatesGPS[index][0], missionCoordinatesGPS[index][1], goal_utm_y, goal_utm_x, goal_zone);
	gps_common::LLtoUTM(map_latitude, map_longitude, map_utm_y, map_utm_x, map_zone);

	double next_goal_x = goal_utm_x - map_utm_x;
	double next_goal_y = goal_utm_y - map_utm_y;

	nextPosition_ = geometry_msgs::Point();
	nextPosition_.x = next_goal_x;
	nextPosition_.y = next_goal_y;
	nextPosition_.z = 0.0;

	ROS_INFO("Next point is %d (%f, %f, %f)", index, nextPosition_.x, nextPosition_.y,
			nextPosition_.z);

	nextOrientation_ = geometry_msgs::Quaternion();
	nextOrientation_.x = 0.0;
	nextOrientation_.y = 0.0;
	nextOrientation_.z = 0.0;
	nextOrientation_.w = 1.0;
}



/**
 *\brief Calculate next goal
 */
void MissionServer::getNextGoal(/*tf::TransformListener* listener*/ ) {

	ROS_INFO("Next orientation is (%f, %f, %f, %f)", nextOrientation_.x,
			nextOrientation_.y, nextOrientation_.z, nextOrientation_.w);

	calculateGoal(missionIndex);		

	int maxMission;

	if(simulate){
		maxMission = sizeof(missionCoordinatesGPS_coenhaven)/sizeof(missionCoordinatesGPS_coenhaven[0]);
	}else{
		maxMission = sizeof(missionCoordinatesGPS)/sizeof(missionCoordinatesGPS[0]);
	}

	ROS_INFO("Max mission index found: %d", maxMission);

	//mission index, start at 0 and end at (maxMission-1)
	missionIndex = (missionIndex + 1) % maxMission;
}

