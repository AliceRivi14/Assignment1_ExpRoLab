#! /usr/bin/env python

import random
import roslib
import time
import rospy
import actionlib
from std_srvs.srv import *
from armor_api.armor_client import ArmorClient

Active = False

Path = 'ERL_WS/src/assignment1/ontology/Map.owl'
IRI = 'http://bnc/exp-rob-lab/2022-23'

def LoadMap():

    rospy.loginfo('I NEED A TOPOLOGICAL MAP')

    # Load ontology
    Armor_Client_ID = 'User'
    Armor_ReferenceName = 'Ref'
    Armor_Client = ArmorClient(Armor_Client_ID, Armor_ReferenceName)
    Armor_Client.utils.load_ref_from_file(Path, IRI, buffered_manipulation=False, reasoner='PELLET', buffered_reasoner=False, mounted=False)

    rospy.loginfo('MAP BUILT')

# Service callback
def Mapping_Switch(req):
    global Active

    Active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'TOPOLOGICAL_MAP state'
    return res

def main():
    # Initialisation node
    rospy.init_node('Mapping')
    global Mapping_Client
    global Active

    srv = rospy.Service('/Mapping_Switch', SetBool, Mapping_Switch)

    while not rospy.is_shutdown():
        if Active == False:
            rospy.loginfo('...')
            continue
        else:
            rospy.loginfo('MAPPING')
            LoadMap()

        # Wait for ctrl-c to stop the application
        rospy.spin()

if __name__ == "__main__":
    main()
