#! /usr/bin/env python

"""
.. module:: RandomMovement
    :platform: Unix
    :synopsis: Python module for robot random movement
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the RANDOM_MOVEMENT state of the finite state machine FSM.

Through this node, the movement of the robot in random positions is simulated.

Client:
    ArmorClient

    MoveBaseAction

Service:
    /Mapping_Switch to active the RANDOM_MOVEMENT state

    MoveBaseGoal

"""

import random
import roslib
import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import *
from assignment1.srv import BatteryLow, BatteryLowResponse
from armor_api.armor_client import ArmorClient

import sys
sys.path.append('~/ERL_WS/src/assignment1/source/script')
import Functions
from Functions import MoveRobot, Destination
import StateMachine
from StateMachine import Battery_State

Active = False
B_Low = False

Robot = 'Robot1'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)

def MoveBaseA():
    """
    Function to provide an implementation of an action which, given a position
    goal, will attempt to reach it.

    If the position is not reached within a certain time (3.0 seconds) or if the
    signal of battery low is sent, the goal is cancelled.
    """
    global B_Low, Target

    MBClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = random.uniform(0.0, 10.0)
    goal.target_pose.pose.position.y = random.uniform(0.0, 10.0)
    MBClient.send_goal(goal)

    if MBClient.wait_for_result(rospy.Duration(3.0)) or B_Low == False:
        rospy.loginfo(f'Position ({goal.target_pose.pose.position.x},{goal.target_pose.pose.position.y}) reached')
        Target = Destination()
        MoveRobot(Target)
    else:
        MBClient.cancel_goal()
        print('GOAL CANCELLED \n')

# Service callback
def Movement_Switch(req):
    """
    Service callback.

    Args:
        req (bool): for enabling/disabling the service related to moving simulation

    Returns:
        res.success (bool): indicates successful run of triggered service

        res.message (string): informational
    """
    global Active, res

    Active = req.data
    res = SetBoolResponse()
    res.message = 'RANDOM_MOVEMENT state'
    res.success = True # Service enable
    return res

def main():
    """
    This function initializes the ROS node and service.

    When the service /Mapping_Switch is called, random movement is simulated.
    """
    global Movement_Client
    global Active

    # Initialisation node
    rospy.init_node('RandomMovement')

    # Initialisation service
    srv = rospy.Service('/Movement_Switch', SetBool, Movement_Switch)
    srv = rospy.Service('/B_Switch', BatteryLow, Battery_State)

    while not rospy.is_shutdown():
        if Active == False:
            continue
        else:
            rospy.loginfo('I AM LOOKING FOR A LOCATION')
            MoveBaseA()
            Active = False

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
