#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import nautonomous_operation_action.msg 
from geometry_msgs.msg import Pose2D

manualPath = False

def mission_plan_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('mission_server', nautonomous_operation_action.msg.MissionPlanAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    
    if manualPath: 
        poses = [Pose2D(626748, 5807649, 0), Pose2D(626765, 5807638, 0), Pose2D(626776, 5807655, 0), Pose2D(626759, 5807666, 0)]
    
    else:
        poses = [Pose2D(628531, 5805040, 0), Pose2D(629444, 5803420, 0)]

    mac = "mac" 
    token = "token"
    action1 = nautonomous_operation_action.msg.OperationPlan(uuid = None, name = "test", path = poses, operationActions = None, automaticPlanning = not manualPath)
    actions = [action1]

    goal = nautonomous_operation_action.msg.MissionPlanGoal(mac = mac, token = token, operationPlan = actions)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('mission_plan_client')
        manualPath = rospy.get_param('~ManualOperation', True)
        print "Manual operation " + str(manualPath)
        result = mission_plan_client()
        print("Result:" + str(result.result.progression) + " " + str(result.result.status))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")