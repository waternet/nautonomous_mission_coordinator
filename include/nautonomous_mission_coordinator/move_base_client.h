/*
 * ActionManager.h
 *
 *  Created on: Mar 19, 2016
 *      Author: zeeuwe01
 */

#ifndef MOVEBASEACTION_H_
#define MOVEBASEACTION_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <nautonomous_mission_msgs/MissionPlanAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class MoveBaseClient{
private:
	MoveBaseActionClient *ac;
public:
	MoveBaseClient();
	~MoveBaseClient();
	int requestGoal(geometry_msgs::Pose2D pose2d);
};

#endif /* MOVEBASEACTION_H_ */
