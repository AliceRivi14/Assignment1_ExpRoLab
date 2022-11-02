#!/usr/bin/env python

import roslib
import rospy
import time
import random
from std_srvs.srv import *
from assignment1.srv import BatteryLow, BatteryLowResponse
from armor_api.armor_client import ArmorClient

import sys
sys.path.append('~/assignment1/script/RandomMovement')
import RandomMovement
from RandomMovement import CleanList

B_Client = None

Active = False
B_Time = random.uniform(5.0, 10.0)
IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)

def MoveRobot(Location):

    Old_Loc = Armor_Client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', Robot])
    Old_Loc = CleanList(Old_Loc)[0]
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', Robot, Location, Old_Loc])
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', Location]))[0])
    New = str(round(time.time()))
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['visitedAt', Location, 'Long', New, Old])
    rospy.loginfo(f' Go to location {Location}')
    Update_Time()

def Update_Time():

    Armor_Client.call('REASON', '', '', [''])
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['now', Robot]))[0])
    New = str(round(time.time()))
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['now', Robot, 'Long', New, Old])

# Service callback
def Battery_Switch(req):
    global Active

    Active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'ROOM_E state'
    return res


def main():
    # Initialisation node
    rospy.init_node('Battery')
    global B_Client
    global Active, B_Time

    srv = rospy.Service('/Battery_Switch', SetBool, Battery_Switch)
    B_Client = rospy.ServiceProxy('B_Switch', BatteryLow)

    while not rospy.is_shutdown():
        if Active == False:
            rospy.loginfo('NO CHARGING')
            time.sleep(B_Time)
            resp = B_Client(True)
            rospy.loginfo('I NEED TO RECHARGE')
            continue
        else:
            rospy.loginfo('CHARGING')
            rospy.loginfo('LOW BATTERY')
            MoveRobot('E')
            time.sleep(B_Time)
            rospy.loginfo(f'BATTERY RECHARGED in {B_Time} SECONDS')

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
