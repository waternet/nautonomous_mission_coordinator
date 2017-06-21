#ifndef OPERATIONMANAGER_H_
#define OPERATIONMANAGER_H_

#include "ros/ros.h"

#include "nautonomous_operation_action/mission_server.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::Pose2D nextPosition_;

void callbackCropper(const std_msgs::Float32MultiArray& msg);

bool simulate;
double map_latitude, map_longitude;


class MissionServer;
MissionServer *server;

MoveBaseActionClient *moveBase;

#endif /* OPERATIONMANAGER_H_ */

