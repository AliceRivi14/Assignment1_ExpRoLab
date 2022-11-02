#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from assignment1.srv import BatteryLow, BatteryLowResponse
from std_srvs.srv import *

Battery_Client = None
Movement_Client = None
Mapping_Client = None
State_ = 1

# 1 - TOPOLOGICAL_MAP state
# 2 - RANDOM_MOVEMENT state
# 3 - ROOM_E state

# Function for interacting with different nodes related to FSM states
def ChangeState(State):
    global Battery_Client, Movement_Client, Mapping_Client
    global State_
    State_ = State
    if State_ == 1:
        resp = Mapping_Client(True)
        resp = Battery_Client(False)
        resp = Movement_Client(False)
    if State_ == 2:
        resp = Mapping_Client(False)
        resp = Battery_Client(False)
        resp = Movement_Client(True)
    if State_ == 3:
        resp = Mapping_Client(False)
        resp = Battery_Client(True)
        resp = Movement_Client(False)


def Action():
    global B_Low

    if B_Low == True:
        B_Low = False
        return 'b_low'
    else:
        B_Low = False
        return 'move'

def Battery_State(req):
    global B_Low

    B_Low = req.B_Low
    res = BatteryLowResponse()
    res.B_State = True
    rospy.loginfo('RECHARGE REQUEST')
    return res

# State TOPOLOGICAL_MAP
class TOPOLOGICAL_MAP(smach.State):
    def __init__(self):
        # Initialisation function
        smach.State.__init__(self,
                             outcomes = ['map_OK',
                                         'move', 'b_low',])
    def execute(self, userdata):
        rospy.loginfo('Executing state TOPOLOGICAL_MAP')
        ChangeState(1)
        time.sleep(5)
        return 'map_OK'

# State RANDOM_MOVEMENT
class RANDOM_MOVEMENT(smach.State):
    def __init__(self):
        #Initialisation function
        smach.State.__init__(self,
                            outcomes = ['b_low',
                                        'map_OK', 'move'])
    def execute(self, userdata):
        rospy.loginfo('Executing state RANDOM_MOVEMENT')
        ChangeState(2)
        time.sleep(5)
        return Action()

# State ROOM_E
class ROOM_E(smach.State):
    def __init__(self):
        #Initialisation function
        smach.State.__init__(self,
                            outcomes = ['move',
                                        'b_low', 'map_OK'])
    def execute(self, userdata):
        rospy.loginfo('Executing state ROOM_E')
        ChangeState(3) # FUNZIONA SOLO LA PRIMA VOLTA
        return Action()

def main():
    global Battery_Client, Movement_Client, Mapping_Client

    # Initialisation node
    rospy.init_node('Robot_State_Machine')

    #
    Battery_Client = rospy.ServiceProxy('/Battery_Switch', SetBool)
    Movement_Client = rospy.ServiceProxy('/Movement_Switch', SetBool)
    Mapping_Client = rospy.ServiceProxy('/Mapping_Switch', SetBool)
    srv = rospy.Service('B_Switch', BatteryLow, Battery_State)

    # Create a SMACH state machine
    SM = smach.StateMachine(outcomes = ['Container'])
    # Open the Container
    with SM:
        # Add states to the Container
        # Initial state
        smach.StateMachine.add('TOPOLOGICAL_MAP', TOPOLOGICAL_MAP(),
                               transitions = {'map_OK': 'RANDOM_MOVEMENT',
                                              'b_low': 'ROOM_E',
                                              'move': 'TOPOLOGICAL_MAP'})

        smach.StateMachine.add('ROOM_E', ROOM_E(),
                               transitions = {'move': 'RANDOM_MOVEMENT',
                                              'b_low': 'ROOM_E',
                                              'map_OK': 'ROOM_E'})

        smach.StateMachine.add('RANDOM_MOVEMENT', RANDOM_MOVEMENT(),
                               transitions = {'b_low': 'ROOM_E',
                                              'move': 'RANDOM_MOVEMENT',
                                              'map_OK': 'RANDOM_MOVEMENT'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('Introspection', SM, '/SM_ROOT')
    sis.start() # Visualization

    # Execute the state machine
    outcome = SM.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
