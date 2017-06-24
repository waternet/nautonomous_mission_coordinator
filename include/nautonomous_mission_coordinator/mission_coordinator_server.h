#ifndef MISSIONSERVER_H_
#define MISSIONSERVER_H_

#include <vector>

#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Pose2D.h>

#include <nautonomous_map_msgs/CropMapPoints.h>
#include <nautonomous_map_msgs/MapLoader.h>

#include <nautonomous_mission_coordinator/move_base_client.h>
#include <nautonomous_mission_msgs/MissionPlanAction.h>

#include <nautonomous_routing_msgs/PathfinderVaarkaart.h>

typedef actionlib::SimpleActionServer<nautonomous_mission_msgs::MissionPlanAction> MissionCoordinatorActionServer;

class MissionCoordinatorServer
{
private:
  ros::NodeHandle nh;

  MissionCoordinatorActionServer missionCoordinatorActionServer;

  std::vector<geometry_msgs::Pose2D> path;

  std::basic_string<char> image_file_name;
	std::basic_string<char> config_file_name;

  MoveBaseClient moveBase;
  
  nautonomous_mission_msgs::MissionPlanFeedback missionPlanFeedback;
  nautonomous_mission_msgs::MissionPlanResult missionPlanResult;

  nautonomous_mission_msgs::OperationPlan current_operation;

public:
  MissionCoordinatorServer(ros::NodeHandle node_handle, std::string name);
  ~MissionCoordinatorServer();

    /**
  *\brief Coordinate Routing from the Vaarkaart using a service request
  *\param std::vector<geometry_msgs::Pose2D> &path
  *\return success
  */
  bool coordinateRoutingVaarkaart();
  /**
  *\brief Coordinate map cropping
  *\param std::basic_string<char> &image_file_name, std::basic_string<char> &config_file_name
  *\return success
  */
  bool coordinateMapCropping();

  /**
  *\brief Coordinate map server
  *\param std::basic_string<char> &image_file_name, std::basic_string<char> &config_file_name
  *\return success
  */
  bool coordinateMapServer();

  /**
  *\brief Coordinate move base goal
  *\param std::vector<geometry_msgs::Pose2D> &path
  *\return success
  */
  bool coordinateMoveBaseGoal();

  /**
  *\brief Empty constructor for MissionCoordinator
  *\param nautonomous_operation_action::MissionGoalConstPtr &goal
  *\return
  */
  void coordinateMission(const nautonomous_mission_msgs::MissionPlanGoalConstPtr &goal);

};

#endif /* MISSIONSERVER_H_ */
