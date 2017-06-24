#ifndef MISSIONCOORDINATORNODE_H_
#define MISSIONCOORDINATORNODE_H_

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>

#include <nautonomous_mission_coordinator/mission_server.h>
#include <nautonomous_mission_coordinator/move_base_client.h>

#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Int32.h"

#include <tf/transform_listener.h>

MissionServer *server;

MoveBaseClient *moveBase;

#endif /* MISSIONCOORDINATORNODE_H_ */

