#include "../include/nautonomous_operation_action/operation_manager.h"

using namespace std;


/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "move_base_action_client");
	ros::NodeHandle nh;

	MissionServer server(nh,"mission_action");
	MoveBaseActionClient moveBase = MoveBaseActionClient();

	ros::Rate r(1);
	r.sleep();

	ros::spinOnce();
	while(ros::ok()){
		ROS_INFO("test...");
		//Simulate goal order (position and orientation)
		server.getNextGoal();
		moveBase.requestGoal(server.nextPosition_, server.nextOrientation_);

		r.sleep();
		ros::spinOnce();
	}
}
