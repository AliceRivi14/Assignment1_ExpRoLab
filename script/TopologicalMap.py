#! /usr/bin/env python

"""
.. module:: TopologicalMap
    :platform: Unix
    :synopsis: Python module for topologic map construction
.. moduleauthor:: Alice Rivi S5135011@studenti.unige.it

ROS node for implementing the TOPOLOGICAL_MAP state of the finite state machine FSM

Client:
    ArmorClient

Service:
    /Mapping_Switch to active the TOPOLOGICAL_MAP state

"""

import random
import roslib
import time
import rospy
import actionlib
from std_srvs.srv import *
from armor_api.armor_client import ArmorClient

import sys
sys.path.append('~/ERL_WS/src/assignment1/source/script')
import Functions
from Functions import CleanList

Active = False

Path = 'ERL_WS/src/assignment1/ontology/Map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'

Armor_Client_ID = 'User'
Armor_ReferenceName = 'Ref'
Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)

def LoadMap():
    """
    Function to load the topological map using the aRMOR client.
    """
    rospy.wait_for_service('armor_interface_srv')
    # Load ontology
    Armor_Client.utils.load_ref_from_file(Path, IRI, buffered_manipulation=False, reasoner='PELLET', buffered_reasoner=False, mounted=False)

    Rooms = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['ROOM']))
    Corridors = CleanList(Armor_Client.call('QUERY', 'IND', 'CLASS', ['CORRIDOR']))

    rospy.loginfo('MAP BUILT')

# Service callback
def Mapping_Switch(req):
    """
    Service callback

    Args:
        req(bool): for enabling/disabling the service related to mappig simulation

    Returns:
        res.success (bool): indicates successful run of triggered service

        res.message (string): informational
    """
    global Active, res

    Active = req.data
    res = SetBoolResponse()
    res.message = 'TOPOLOGICAL_MAP state'
    res.success = True # Service enable
    return res

def main():
    """
    This function initializes the ROS node and service.

    When the service /Mapping_Switch is called, map loading is simulated.
    """
    global Mapping_Client
    global Active

    # Initialisation node
    rospy.init_node('TopologicalMap')

    # Initialisation service
    srv = rospy.Service('/Mapping_Switch', SetBool, Mapping_Switch)

    while not rospy.is_shutdown():
        if Active == False:
            continue
        else:
            rospy.loginfo('I NEED A TOPOLOGICAL MAP')
            LoadMap()
            Active = False

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
