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

#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

class MoveBaseClient{
private:
	MoveBaseActionClient* move_base_action_client_;
public:
	MoveBaseClient();
	~MoveBaseClient();
	bool requestGoal(const geometry_msgs::Pose2D pose2d);
};

#endif /* MOVEBASEACTION_H_ */
