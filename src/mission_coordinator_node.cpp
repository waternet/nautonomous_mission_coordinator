#include <nautonomous_mission_coordinator/mission_coordinator_node.h>

/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mission_coordinator_node");
	ros::NodeHandle nh;

	// Map server
	ros::Publisher pub_new_map = nh.advertise<std_msgs::Int32>("/map/cropper/new_map", 5);

	/* Autonomous */
	MissionCoordinatorServer missionCoordinatorServer(nh,"/mission/coordinator/server");
	
	ros::spin();

}
