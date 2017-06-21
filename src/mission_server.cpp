#include "../include/nautonomous_operation_action/mission_server.h"
#include <gps_common/conversions.h>

/**
 *\brief Constructor for MissionServer
 *\param ros::NodeHandle nh_
 *\param string name
 *\return MissionServer
 */
MissionServer::MissionServer(ros::NodeHandle nh_, std::string name) :
		as_(nh_, name.c_str(), boost::bind(&MissionServer::executeCB, this, _1),
				false) {
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
void MissionServer::executeCB(const nautonomous_operation_action::MissionPlanGoalConstPtr &goal) {

	// helper variables
	nautonomous_operation_action::MissionPlanFeedback feedback_;
    nautonomous_operation_action::MissionPlanResult result_;

	ROS_INFO("Mission Server callback");
	//Check token: TODO

	ros::NodeHandle nh;
    
	MoveBaseActionClient moveBase = MoveBaseActionClient();

	ros::Rate r(1);
	bool success = true;
	int operationIndex = 1;

	// For each operation in the mission plan execute the operation.
	ROS_INFO("Operations size: %i", goal->operationPlan.size());
	for(std::vector<nautonomous_operation_action::OperationPlan>::const_iterator operation_iterator = goal->operationPlan.begin(); operation_iterator != goal->operationPlan.end(); ++operation_iterator)
	{
		nautonomous_operation_action::OperationPlan current_operation = *operation_iterator;
		ROS_INFO("Operation: %i", operationIndex);

		//Mission Planner
		ROS_INFO("Starting path assembly ... ");
		std::vector<geometry_msgs::Pose2D> path;
		if(current_operation.automaticPlanning){
			ROS_INFO("Automatic Planning initiated...");
			ros::ServiceClient pathfinder_client = nh.serviceClient<nautonomous_navigation_pathfinder::FindPathAmsterdamCanals>("find_path_amsterdam_canals");
			
			nautonomous_navigation_pathfinder::FindPathAmsterdamCanals pathfinder_srv;
			pathfinder_srv.request.goalEasting = current_operation.path[1].x;
			pathfinder_srv.request.goalNorthing = current_operation.path[1].y;
			pathfinder_srv.request.testStartEasting = current_operation.path[0].x;
			pathfinder_srv.request.testStartNorthing = current_operation.path[0].y;

			ROS_INFO("Performing pathfinder request...");		
			if(pathfinder_client.call(pathfinder_srv)) {
				// copy the path from the pathfinder to the path vector	
				int pathIndex = 1;
				for(std::vector<geometry_msgs::Pose2D>::const_iterator path_iterator = pathfinder_srv.response.pathLocations.begin(); path_iterator != pathfinder_srv.response.pathLocations.end(); ++path_iterator)
				{
					path.push_back(*path_iterator);
				}
				
			} else {
				ROS_ERROR("Failed request for path finder");
				success = false;
				break;
			}
		} else {
			// copy the path from the action to the path vector	
			int pathIndex = 1;
			for(std::vector<geometry_msgs::Pose2D>::const_iterator path_iterator = current_operation.path.begin(); path_iterator != current_operation.path.end(); ++path_iterator)
			{
				path.push_back(*path_iterator);
			}
		}
		ROS_INFO("... finished path assembly.");


		//Cropper
		ros::ServiceClient map_cropper_client = nh.serviceClient<nautonomous_map_cropper::CropMapPoints>("crop_map_points");
			
		nautonomous_map_cropper::CropMapPoints map_cropper_srv;
		map_cropper_srv.request.pathLocations = path;
		map_cropper_srv.request.operation_name = current_operation.name;

		std::basic_string<char> image_file_name;
		std::basic_string<char> config_file_name;
		ROS_INFO("Calling map cropper ... ");
		if(map_cropper_client.call(map_cropper_srv)){
			image_file_name = map_cropper_srv.response.image_file_name;
			config_file_name = map_cropper_srv.response.config_file_name;

			ROS_INFO("Created new map called %s %s", image_file_name.c_str(), config_file_name.c_str());
		} else {
			ROS_ERROR("Failed request for map cropper");
			success = false;
			break;
		}
		ROS_INFO("... finished map cropper.");


		//Map Server
		ros::ServiceClient map_server_client = nh.serviceClient<nautonomous_msgs::MapLoader>("map_server/load_map");
		
		nautonomous_msgs::MapLoader map_server_srv;
		map_server_srv.request.image_file_name = image_file_name;
		map_server_srv.request.config_file_name = config_file_name;

		ROS_INFO("Calling map server ... %s %s", map_server_srv.request.image_file_name.c_str(), map_server_srv.request.config_file_name.c_str());
		if(map_server_client.call(map_server_srv)){
			std::basic_string<char> status = map_server_srv.response.status;
			ROS_INFO("Loaded new map with status %s", status.c_str());
		} else {
			ROS_ERROR("Failed request for map server");
			success = false;
			break;
		}
		ROS_INFO("... finished map server.");


		// Request goals using move base navigation stack
		int numPath = path.size();
		ROS_INFO("Number of path segments for move base %i", numPath);
		for(int pathIndex = 0; pathIndex < numPath; pathIndex++){
			ROS_INFO("Route index: %i", pathIndex);

			moveBase.requestGoal(path.at(pathIndex));

			// push_back the seeds for the mission status
			feedback_.feedback.progression = (int) ((pathIndex+1)/numPath); //arbitrary value for now TODO
			feedback_.feedback.status = "Ok";
			as_.publishFeedback(feedback_);

			r.sleep();
			ros::spinOnce();
			
		}

	}

	if (success) {
		result_.result.progression = 100;
		result_.result.status = feedback_.feedback.status;

		// set the action state to succeeded
		as_.setSucceeded(result_);
	} else {
		result_.result.progression = 0;
		result_.result.status = "Failed pathfinder request";
    
		// set the action state to succeeded
		as_.setAborted(result_);
	}
}
