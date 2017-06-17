///*
// * MissionServer.h
// *
// *  Created on: Apr 3, 2016
// *      Author: zeeuwe01
// */
//
#ifndef MISSIONSERVER_H_
#define MISSIONSERVER_H_

#include "ros/ros.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include "nautonomous_operation_action/move_base_action_client.h"
#include <nautonomous_operation_action/MissionPlanAction.h>
#include <nautonomous_map_cropper/CropMapPoints.h>
#include <nautonomous_msgs/MapLoader.h>
#include "nautonomous_navigation_pathfinder/FindPathAmsterdamCanals.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <vector>

typedef actionlib::SimpleActionServer<nautonomous_operation_action::MissionPlanAction> Server;

class MissionServer
{

public:

  MissionServer(ros::NodeHandle nh_, std::string name);

  ~MissionServer(void);

  void executeCB(const nautonomous_operation_action::MissionPlanGoalConstPtr &goal);

  actionlib::SimpleActionServer<nautonomous_operation_action::MissionPlanAction> as_;

};
#endif /* MISSIONSERVER_H_ */
