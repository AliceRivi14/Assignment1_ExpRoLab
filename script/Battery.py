#!/usr/bin/env python

"""
.. module:: Battery
    :platform: Unix
    :synopsis: Python module for battery control
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the ROOM_E state of the finite state machine FSM

Client:
    /B_Switch to communicate the need for recharging the battery

    ArmorClient

Service:
    /Recharging_Switch to active the ROOM_E state

"""

import roslib
import rospy
import time
import random
from std_srvs.srv import *
from assignment1.srv import BatteryLow, BatteryLowResponse
from armor_api.armor_client import ArmorClient

import sys
#sys.path.append('~/ERL_WS/src/assignment1/source/script')
import Functions
from Functions import MoveRobot

B_Client = None
Active = False
B_Time = random.uniform(0.5, 2.5)

# Service callback
def Battery_Switch(req):
    """
    Service callback.

    Args:
        req (bool): for enabling/disabling the service related to battery charging simulation

    Returns:
        res.success (bool): indicates successful run of triggered service

        res.message (string): informational
    """
    global Active, res

    Active = req.data
    res = SetBoolResponse()
    res.message = 'ROOM_E state'
    res.success = True # Service enable
    return res

def main():
    """
    This function initializes the ROS node, client and service.

    A message is sent to the service /B_switch every random seconds to notify the need for recharging.

    When the service /Recharging_Switch is called, battery charging is simulated.
    """
    global B_Client
    global Active, B_Time

    # Initialisation node
    rospy.init_node('Battery')

    # Initialisation clients and service
    srv = rospy.Service('/Recharging_Switch', SetBool, Battery_Switch)
    B_Client = rospy.ServiceProxy('/B_Switch', BatteryLow)

    while not rospy.is_shutdown():
        if Active == False:
            time.sleep(random.uniform(5.0, 10.0))
            resp = B_Client(True) # Recharging required
            rospy.loginfo('I NEED TO RECHARGE')
            continue
        else:
            rospy.loginfo('BATTERY LOW')
            MoveRobot('E')
            time.sleep(B_Time)
            # NON ARRIVA MAI QUA A CAUSA DEL MoveBaseAction
            rospy.loginfo(f'Battery recharged in {B_Time} seconds')
            resp = B_Client(False)
            Active = False # PER PROVARE

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
