#include "../include/nautonomous_operation_action/operation_manager.h"
#include "../include/nautonomous_navigation_planner/AddTwoInts.h"

using namespace std;


/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "move_base_action_client");
	ros::NodeHandle nh;

	/* Autonomous */
	ros::ServiceClient client = nh.serviceClient<nautonomous_navigation_planner::AddTwoInts>("add_two_ints");
	nautonomous_navigation_planner::AddTwoInts srv;
	srv.request.c = 52.36905;
	srv.request.d = 4.89248;

	MissionServer server(nh,"mission_action");
	MoveBaseActionClient moveBase = MoveBaseActionClient();

	ros::Rate r(1);
	r.sleep();

	ros::spinOnce();
	while(ros::ok()){
		
		//Simulate goal order (position and orientation)

		//Manual coordinates 
		
		/*
		server.getNextGoal();
		moveBase.requestGoal(server.nextPosition_, server.nextOrientation_);
		*/

		/* Autonomous */

		if(client.call(srv)) {
			nextPosition_ = geometry_msgs::Point();
			nextPosition_.x = srv.response.x;
			nextPosition_.y = srv.response.y;
			nextPosition_.z = 0;
	
			nextOrientation_ = geometry_msgs::Quaternion();
			nextOrientation_.x = 0.0;
			nextOrientation_.y = 0.0;
			nextOrientation_.z = srv.response.lat;
			nextOrientation_.w = srv.response.lon;
			moveBase.requestGoal(nextPosition_, nextOrientation_);
		}

		
		
		r.sleep();
		ros::spinOnce();
	}
}
