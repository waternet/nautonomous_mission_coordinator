#include <nautonomous_mission_coordinator/mission_coordinator_server.h>

/**
 *\brief Constructor for MissionServer
 *\param ros::NodeHandle nh_
 *\param string name
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::MissionCoordinatorServer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle, std::string name) :
		mission_coordinator_action_server_(node_handle, name.c_str(), boost::bind(&MissionCoordinatorServer::coordinateMission, this, _1),
				false) 
{
	mission_coordinator_action_server_.start();

	node_handle_ = node_handle;

	private_node_handle.param("routing_enabled", routing_enabled_, false);
	private_node_handle.param("map_enabled", map_enabled_, false);
	private_node_handle.param("navigate_enabled", navigate_enabled_, true);

	move_base_client_ = new MoveBaseClient();
}

/**
 *\brief Empty constructor for MissionCoordinatorServer
 *\param 
 *\return MissionCoordinatorServer
 */
MissionCoordinatorServer::~MissionCoordinatorServer(void) 
{
	if(move_base_client_){
		delete move_base_client_;
		move_base_client_ = nullptr;
	}
}

/**
 *\brief Coordinate Routing from the Vaarkaart using a service request
 *\param std::vector<geometry_msgs::Pose2D> &path, nautonomous_mission_msgs::OperationPlan current_operation_
 *\return success
 */
bool MissionCoordinatorServer::coordinateRoutingVaarkaart()
{
	route_.clear();
	//If the operation should be automatically planned, we use the vaarkaart to do so.
	if(current_operation_.automatic_routing)
	{
		//Create routing request for the vaarkaart.
		nautonomous_routing_msgs::Route route_srv;
		route_srv.request.start = current_operation_.route[0];
		route_srv.request.destination = current_operation_.route[1];

		// Create the routing 
		ros::ServiceClient vaarkaart_client = node_handle_.serviceClient<nautonomous_routing_msgs::Route>("route_service");	
		if(vaarkaart_client.call(route_srv)) 
		{
			// Copy the path from the routing to the path vector	
			for(std::vector<geometry_msgs::Pose2D>::const_iterator route_iterator = route_srv.response.route.begin(); route_iterator != route_srv.response.route.end(); ++route_iterator)
			{
				route_.push_back(*route_iterator);
			}
		} 
		else 
		{
			ROS_ERROR("Failed request for routing");
			return false;
		}
	} 
	else 
	// We do not have to plan automatically and assume the action path is already the correct final path.
	{
		// Copy the path from the action to the path vector	
		for(std::vector<geometry_msgs::Pose2D>::const_iterator route_iterator = current_operation_.route.begin(); route_iterator != current_operation_.route.end(); ++route_iterator)
		{
			route_.push_back(*route_iterator);
		}
	}

	return true;
}

/**
 *\brief Coordinate map cropping
 *\param std::vector<geometry_msgs::Pose2D> &path,  std::basic_string<char> &config_name
 *\return success
 */
bool MissionCoordinatorServer::coordinateMapCropping()
{
	// Create a map cropping service request
	nautonomous_map_msgs::Crop crop_srv;
	crop_srv.request.route = route_;
	crop_srv.request.name = current_operation_.name;

	// Execute map cropping service call
	ros::ServiceClient map_cropper_client = node_handle_.serviceClient<nautonomous_map_msgs::Crop>("crop_service");
	if(map_cropper_client.call(crop_srv))
	{
		config_name_ = crop_srv.response.config_name;
	} 
	else 
	{
		ROS_ERROR("Failed request for map cropper");
		return false;
	}

	return true;
}

/**
 *\brief Coordinate map server
 *\param std::basic_string<char> &config_name
 *\return success
 */
bool MissionCoordinatorServer::coordinateMapServer()
{
	// Create a map server service request
	nautonomous_map_msgs::Load load_srv;
	load_srv.request.config_name = config_name_;

	// Execute map server request
	ros::ServiceClient map_server_client = node_handle_.serviceClient<nautonomous_map_msgs::Load>("load_service");
	if(map_server_client.call(load_srv))
	{
		std::string status = load_srv.response.status;
		//TODO what to do with the status.
	} 
	else 
	{
		ROS_ERROR("Failed request for map server");
		return false;
	}

	return true;
}

/**
 *\brief Coordinate move base goal
 *\param std::vector<geometry_msgs::Pose2D> &path
 *\return success
 */
bool MissionCoordinatorServer::coordinateMoveBaseGoal()
{
	int route_size = route_.size();
	for(int route_index = 0; route_index < route_size; route_index++)
	{
		if(!move_base_client_->requestGoal(route_.at(route_index)))
		{
			return false;
		}

		// Publish the current progression.
		mission_plan_feedback_.feedback.progression = (int) ((route_index + 1) / route_size); //arbitrary value for now TODO
		mission_plan_feedback_.feedback.status = "Ok";
		mission_coordinator_action_server_.publishFeedback(mission_plan_feedback_);

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
	//Check token: TODO
	//authenticate

	bool success = true;

	// For each operation in the mission plan execute the operation.
	for(std::vector<nautonomous_mission_msgs::OperationPlan>::const_iterator operation_pointer = goal->operations.begin(); operation_pointer != goal->operations.end(); ++operation_pointer)
	{
		current_operation_ = *operation_pointer;

		//Mission Planner
		if(!coordinateRoutingVaarkaart() && routing_enabled_)
		{
			success = false;
			break;
		}

		//Cropper
		if(!coordinateMapCropping() && map_enabled_)
		{
			success = false;
			break;
		}

		//Map Server
		if(!coordinateMapServer() && map_enabled_)
		{
			success = false;
			break;
		}

		// Request goals using move base navigation stack
		if(!coordinateMoveBaseGoal() && map_enabled_ && navigate_enabled_)
		{
			success = false;
			break;
		}	

	}

	// Return the progress and status for both success and failed action.
	if (success) 
	{
		mission_plan_result_.result.progression = 100;
		mission_plan_result_.result.status = mission_plan_feedback_.feedback.status;
		// Set the action state to succeeded
		mission_coordinator_action_server_.setSucceeded(mission_plan_result_);
	} 
	else 
	{
		//TODO correct progression calculation
		mission_plan_result_.result.progression = 0;
		mission_plan_result_.result.status = "Failed coordinating mission request";
		// Set the action state to aborted
		mission_coordinator_action_server_.setAborted(mission_plan_result_);
	}
}
