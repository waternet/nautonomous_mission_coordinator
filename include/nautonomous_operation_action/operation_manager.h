#ifndef OPERATIONMANAGER_H_
#define OPERATIONMANAGER_H_

#include "ros/ros.h"
#include "nautonomous_operation_action/move_base_action_client.h"
#include "nautonomous_operation_action/mission_server.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::Point nextPosition_;
geometry_msgs::Quaternion nextOrientation_;

double map_latitude, map_longitude;

#endif /* OPERATIONMANAGER_H_ */

