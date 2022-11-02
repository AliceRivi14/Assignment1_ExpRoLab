#! /usr/bin/env python

import random
import roslib
import time
import rospy
import actionlib
from std_srvs.srv import *
from armor_api.armor_client import ArmorClient

Active = False

IRI = 'http://bnc/exp-rob-lab/2022-23'
Robot = 'Robot1'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)

def CleanList(res):

    List = res.queried_objects
    # Remove IRI
    List =  [Q.replace('<'+IRI+'#', '') for Q in List]
    List =  [Q.replace('>', '') for Q in List]
    # Remove timestamp
    List =  [Q.replace('"', '') for Q in List]
    List =  [Q.replace('^^xsd:long', '') for Q in List]

    return List

def MoveRobot(Location):

    Old_Loc = Armor_Client.call('QUERY', 'OBJECTPROP', 'IND', ['isIn', Robot])
    Old_Loc = CleanList(Old_Loc)[0]
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', Robot, Location, Old_Loc])
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['visitedAt', Location]))[0])
    New = str(round(time.time()))
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['visitedAt', Location, 'Long', New, Old])
    rospy.loginfo(f' Go to {Location}')
    Update_Time()

def Update_Time():

    Armor_Client.call('REASON', '', '', [''])
    Old = str(CleanList(Armor_Client.call('QUERY', 'DATAPROP', 'IND', ['now', Robot]))[0])
    New = str(round(time.time()))
    Armor_Client.call('REPLACE', 'OBJECTPROP', 'IND', ['now', Robot, 'Long', New, Old])


def Destination():

    Urgent = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['URGENT']))
    Urgent = [Idx for Idx in Urgent if Idx[0].lower() == 'R'.lower()]
    Reachable = CleanList(Armor_Client.call('QUERY', 'OBJECTPROP', 'IND', ['canReach', Robot]))
    Urgent = [Value for Value in Urgent if Value in Reachable]

    if Urgent:
        Target = random.choice(Urgent)
    else:
        Target = [Idx for Idx in Reachable if Idx[0].lower() == 'C'.lower()]
        if Target:
            Target = random.choice(Target)
        else:
            Target = random.choice(Reachable)
            if not Target:
                print('ERROR')

    MoveRobot(Target)

    return Target

# Service callback
def Movement_Switch(req):
    global Active

    Active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'RANDOM_MOVEMENT state'
    return res

def Movement():
    global Movement_Client

    rospy.loginfo('I AM LOOKING FOR A LOCATION')
    Destination()
    rospy.loginfo('LOCATION REACHED')


def main():
    # Initialisation node
    rospy.init_node('Movement')
    global Movement_Client
    global Active

    srv = rospy.Service('/Movement_Switch', SetBool, Movement_Switch)

    while not rospy.is_shutdown():
        if Active == False:
            rospy.loginfo('WAITING TO MOVE')
            continue
        else:
            rospy.loginfo('RANDOM MOVEMENT')
            Movement()

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
