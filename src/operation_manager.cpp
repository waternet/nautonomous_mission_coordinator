#include "../include/nautonomous_operation_action/operation_manager.h"


#include "std_msgs/Int32.h"

using namespace std;


/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "move_base_action_client");
	ros::NodeHandle nh;

	// Map server
	ros::Publisher pub_new_map = nh.advertise<std_msgs::Int32>("map_cropper/new_map", 5);

	/* Autonomous */
	ROS_INFO("Request service");
	MissionServer server(nh,"mission_server");
	

	ros::spin();

}
