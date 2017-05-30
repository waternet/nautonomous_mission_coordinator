#include "../include/nautonomous_operation_action/operation_manager.h"
#include "../include/nautonomous_navigation_pathfinder/AddTwoInts.h"

using namespace std;


/**
 * \brief Callback for cropper location. Sets location of center of the cropped map
 * \param 
 */
void callbackCropper(const std_msgs::Float32MultiArray& msg) {

    ROS_INFO("Operation manager callback map location: %f / %f", msg.data[0], msg.data[1]);

    map_latitude = msg.data[0];
    map_longitude = msg.data[1];

	moveBase->cancelGoal();
	//server->calculateGoal(-1);
}


/**
 *\brief Creates MissionServer (mission_server.cpp) and MoveBaseActionClient (move_base_action_client.cpp). Loops through goals from MissionServer
 */
int main(int argc, char** argv){
	ros::init(argc, argv, "move_base_action_client");
	ros::NodeHandle nh;

	nh.getParam("simulate", simulate);

	if(simulate){
		//Coenhaven
		ROS_INFO("Setting map center to coenhaven");
		map_latitude = 52.404434;
	    map_longitude = 4.863055;
	}

	/* Autonomous 
	ros::ServiceClient client = nh.serviceClient<nautonomous_navigation_pathfinder::AddTwoInts>("add_two_ints");
	nautonomous_navigation_pathfinder::AddTwoInts srv;
	srv.request.c = 52.36905;
	srv.request.d = 4.89248;
	*/

	ROS_INFO("Operation man simulate: %d", simulate);

	server = new MissionServer(nh,"mission_action");
	moveBase = new MoveBaseActionClient();

	ros::NodeHandle node;
	ros::Subscriber crop_sub = node.subscribe("map_server/map_data", 10, callbackCropper);

	ros::Rate r(1);
	r.sleep();

	//ros::spinOnce();
	ros::AsyncSpinner spinner(2);
	spinner.start();

	while(ros::ok()){
		
		//Simulate goal order (position and orientation)

		/* Manual coordinates */
		if(map_latitude != 0.0 || map_longitude != 0.0){

			ROS_INFO("Map values: %f / %f", map_latitude, map_longitude);

			ROS_INFO("Manual coordinates");
			server->getNextGoal();
			moveBase->requestGoal(server->nextPosition_, server->nextOrientation_);
		}

		/* Autonomous 

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
		*/

		r.sleep();
		//ros::spinOnce();
	}

	delete server;
	delete moveBase;
}
