#include <nautonomous_mission_coordinator/mission_coordinator_server.h>

/**
 *\brief Constructor for MissionServer
 *\param ros::NodeHandle nh_
 *\param string name
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::MissionCoordinatorServer(ros::NodeHandle node_handle, std::string name) :
		missionCoordinatorActionServer(node_handle, name.c_str(), boost::bind(&MissionCoordinatorServer::coordinateMission, this, _1),
				false) 
{
	missionCoordinatorActionServer.start();

	nh = node_handle;
}

/**
 *\brief Empty constructor for MissionCoordinatorServer
 *\param 
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::~MissionCoordinatorServer(void) 
{
}

/**
 *\brief Coordinate Routing from the Vaarkaart using a service request
 *\param std::vector<geometry_msgs::Pose2D> &path, nautonomous_mission_msgs::OperationPlan current_operation
 *\return success
 */
bool MissionCoordinatorServer::coordinateRoutingVaarkaart()
{
	ROS_INFO("Starting path assembly ... ");
	path.clear();
	//If the operation should be automatically planned, we use the vaarkaart to do so.
	if(current_operation.automaticPlanning)
	{
		ROS_INFO("\tAutomatic Planning initiated...");
		//Create routing request for the vaarkaart.
		nautonomous_routing_msgs::PathfinderVaarkaart vaarkaart_srv;
		vaarkaart_srv.request.goalEasting = current_operation.path[1].x;
		vaarkaart_srv.request.goalNorthing = current_operation.path[1].y;
		vaarkaart_srv.request.testStartEasting = current_operation.path[0].x;
		vaarkaart_srv.request.testStartNorthing = current_operation.path[0].y;

		ROS_INFO("\tPerforming pathfinder request...");	
		// Create the routing 
		ros::ServiceClient vaarkaart_client = nh.serviceClient<nautonomous_routing_msgs::PathfinderVaarkaart>("/routing/vaarkaart/request");	
		if(vaarkaart_client.call(vaarkaart_srv)) 
		{
			// Copy the path from the pathfinder to the path vector	
			int pathIndex = 1;
			for(std::vector<geometry_msgs::Pose2D>::const_iterator path_iterator = vaarkaart_srv.response.pathLocations.begin(); path_iterator != vaarkaart_srv.response.pathLocations.end(); ++path_iterator)
			{
				path.push_back(*path_iterator);
			}
		} 
		else 
		{
			ROS_ERROR("Failed request for path finder");
			return false;
		}
	} 
	else 
	// We do not have to plan automatically and assume the action path is already the correct final path.
	{
		// Copy the path from the action to the path vector	
		int pathIndex = 1;
		for(std::vector<geometry_msgs::Pose2D>::const_iterator path_iterator = current_operation.path.begin(); path_iterator != current_operation.path.end(); ++path_iterator)
		{
			path.push_back(*path_iterator);
		}
	}
	ROS_INFO("... finished path assembly correctly.");
	return true;
}

/**
 *\brief Coordinate map cropping
 *\param std::vector<geometry_msgs::Pose2D> &path, std::basic_string<char> &image_file_name, std::basic_string<char> &config_file_name
 *\return success
 */
bool MissionCoordinatorServer::coordinateMapCropping()
{
	// Create a map cropping service request
	nautonomous_map_msgs::CropMapPoints map_cropper_srv;
	map_cropper_srv.request.pathLocations = path;
	map_cropper_srv.request.operation_name = current_operation.name;

	ROS_INFO("Calling map cropper ... ");
	// Execute map cropping service call
	ros::ServiceClient map_cropper_client = nh.serviceClient<nautonomous_map_msgs::CropMapPoints>("/map/crop/request");
	if(map_cropper_client.call(map_cropper_srv))
	{
		image_file_name = map_cropper_srv.response.image_file_name;
		config_file_name = map_cropper_srv.response.config_file_name;

		ROS_INFO("\tCreated new map called %s %s", image_file_name.c_str(), config_file_name.c_str());
	} 
	else 
	{
		ROS_ERROR("Failed request for map cropper");
		return false;
	}

	ROS_INFO("... finished map cropper correctly.");
	return true;
}

/**
 *\brief Coordinate map server
 *\param std::basic_string<char> &image_file_name, std::basic_string<char> &config_file_name
 *\return success
 */
bool MissionCoordinatorServer::coordinateMapServer()
{
	// Create a map server service request
	nautonomous_map_msgs::MapLoader map_server_srv;
	map_server_srv.request.image_file_name = image_file_name;
	map_server_srv.request.config_file_name = config_file_name;
	ROS_INFO("Calling map server ... %s %s", map_server_srv.request.image_file_name.c_str(), map_server_srv.request.config_file_name.c_str());
	
	// Execute map server request
	ros::ServiceClient map_server_client = nh.serviceClient<nautonomous_map_msgs::MapLoader>("/map/server/load");
	if(map_server_client.call(map_server_srv))
	{
		std::basic_string<char> status = map_server_srv.response.status;
		ROS_INFO("\tLoaded new map with status %s", status.c_str());
	} 
	else 
	{
		ROS_ERROR("Failed request for map server");
		return false;
	}

	ROS_INFO("... finished map server correctly.");
	return true;
}

/**
 *\brief Coordinate move base goal
 *\param std::vector<geometry_msgs::Pose2D> &path
 *\return success
 */
bool MissionCoordinatorServer::coordinateMoveBaseGoal()
{
	ROS_INFO("Calling move base client ...");
	int numPath = path.size();
	
	for(int pathIndex = 0; pathIndex < numPath; pathIndex++)
	{

		ROS_INFO("\tRoute index: %i", pathIndex);
		if(!moveBase.requestGoal(path.at(pathIndex)))
		{
			return false;
		}

		// Publish the current progression.
		missionPlanFeedback.feedback.progression = (int) ((pathIndex+1)/numPath); //arbitrary value for now TODO
		missionPlanFeedback.feedback.status = "Ok";
		missionCoordinatorActionServer.publishFeedback(missionPlanFeedback);

		ros::spinOnce();
	
	}

	return true;
}

/**
 *\brief Empty constructor for MissionCoordinatorServer
 *\param nautonomous_operation_action::MissionGoalConstPtr &goal
 *\return
 */
void MissionCoordinatorServer::coordinateMission(const nautonomous_mission_msgs::MissionPlanGoalConstPtr &goal) 
{
	ROS_INFO("Mission Server callback");
	//Check token: TODO
	//authenticate

    moveBase = MoveBaseClient();

	bool success = true;
	int operationIndex = 1;

	// For each operation in the mission plan execute the operation.
	ROS_INFO("Operations size: %i", (int)goal->operationPlan.size());
	for(std::vector<nautonomous_mission_msgs::OperationPlan>::const_iterator operation_iterator = goal->operationPlan.begin(); operation_iterator != goal->operationPlan.end(); ++operation_iterator)
	{
		current_operation = *operation_iterator;
		ROS_INFO("Operation: %i", operationIndex++);

		//Mission Planner
		if(coordinateRoutingVaarkaart())
		{
			success = false;
			break;
		}

		//Cropper
		if(coordinateMapCropping())
		{
			success = false;
			break;
		}

		//Map Server
		if(coordinateMapServer())
		{
			success = false;
			break;
		}

		// Request goals using move base navigation stack
		if(coordinateMoveBaseGoal())
		{
			success = false;
			break;
		}	

	}

	// Return the progress and status for both success and failed action.
	if (success) 
	{
		missionPlanResult.result.progression = 100;
		missionPlanResult.result.status = missionPlanFeedback.feedback.status;
		// Set the action state to succeeded
		missionCoordinatorActionServer.setSucceeded(missionPlanResult);
	} 
	else 
	{
		missionPlanResult.result.progression = 0;
		missionPlanResult.result.status = "Failed coordinating mission request";
		// Set the action state to aborted
		missionCoordinatorActionServer.setAborted(missionPlanResult);
	}
}
