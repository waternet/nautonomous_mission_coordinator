#ifndef MISSIONSERVER_H_
#define MISSIONSERVER_H_

#include <vector>

#include "ros/ros.h"

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <nautonomous_map_msgs/CropMapPoints.h>
#include <nautonomous_map_msgs/MapLoader.h>

#include <nautonomous_mission_coordinator/move_base_client.h>
#include <nautonomous_mission_msgs/MissionPlanAction.h>

#include <nautonomous_routing_msgs/PathfinderVaarkaart.h>

#include <std_msgs/String.h>

#include <tf/transform_listener.h>


typedef actionlib::SimpleActionServer<nautonomous_mission_msgs::MissionPlanAction> Server;

class MissionServer
{

public:

  MissionServer(ros::NodeHandle nh_, std::string name);
  
  ~MissionServer(void);

  void executeCB(const nautonomous_mission_msgs::MissionPlanGoalConstPtr &goal);

  actionlib::SimpleActionServer<nautonomous_mission_msgs::MissionPlanAction> as_;

};

#endif /* MISSIONSERVER_H_ */
