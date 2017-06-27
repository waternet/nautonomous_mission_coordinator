#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

from geometry_msgs.msg import Pose2D

from nautonomous_mission_msgs.msg import MissionPlanAction, OperationPlan, MissionPlanGoal

manual_routing = False

def mission_plan_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/mission/coordinator/server', MissionPlanAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    operation_name = ""
    if manual_routing: 
        route_poses = [Pose2D(626748, 5807649, 0), Pose2D(626765, 5807638, 0), Pose2D(626776, 5807655, 0), Pose2D(626759, 5807666, 0)]
        operation_name = "coenhaven"
    else:
        route_poses = [Pose2D(628777, 5804903, 0), Pose2D(629444, 5803420, 0)]
        operation_name = "canals"

    mac = "mac" 
    token = "token"
    
    # Create actions
    operation1 = OperationPlan(uuid = None, name = operation_name, route = route_poses, actions = None, automatic_routing = not manual_routing)
    operations = [operation1]

    # Create goal
    goal = MissionPlanGoal(mac = mac, token = token, operations = operations)

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
        rospy.init_node('mission_plan_client_node')
        manual_routing = rospy.get_param('~manual_routing', True)

        rospy.sleep(1)

        print("Requesting mission plan to be executed.")
        result = mission_plan_client()
        print("Result:" + str(result.result.progression) + " " + str(result.result.status))
   
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
