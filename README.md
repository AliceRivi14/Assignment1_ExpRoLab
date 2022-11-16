Behavioural Architechture
================================
**A ROS-based assignment for the Experimental Robotics Laboratory course held at the University of Genoa.**

*Author: Alice Rivi <S5135011@studenti.unige.it>*

Introduction 
-----------------

This software architecture has been developed by [Student Robotics](https://studentrobotics.org) in ROS to simulate a surveillance robot.
The whole software is provided in Python 3.

The software developed uses a [Smach State Machine](http://wiki.ros.org/smach) and builds an ontology with aRMOR, using the [armor_py_api](https://github.com/EmaroLab/armor_py_api).

The scenario involves a survillance robot operating in a 2D indoor environment, without obstacles, made of 4 rooms (R1, R2, R3, R4) and 3 corridors (E, C1, C2).

![Immagine 2022-11-14 173457](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/Immagine%202022-11-14%20173457.png)

The behavior of the robot is divided into 2 phases:

* Phase 1:
1. The robot start in the E location;
2. The robot waits until it receives the information to build the topological map;
3. The robot builts the map.

* Phase 2:
1. The robot moves through locations following a surveillance policy:
    1. It should mainly stay on corridors,
    2. If a reachable room has not been visited for some times it should visit it;
2. The robot moves in a new location, and waits for some times before to visit another location. 
3. When the robot’s battery is low, the robot goes in the E location, and wait for some times before to start again with the above behavior.


ROS Architecture
------------------------
There are four nodes in this software architecture:
* StateMachine
* TopologicalMap
* RandomMovement
* Battery

### StateMachine node 
viewer FOTO

In this node, the 4 classes representing the states of the finite state machine FSM are initialized. Each state is characterised by the initialisation function `__init__(self)` and the function representing the state execution `execute(self, userdata)`.

- `class TOPOLOGICAL_MAP(smach.State)`: Class implementing FSM state concerning the topology map.

    Within the execute function, a call is made to the server connecting the FSM to the TopologicalMap node, and 3 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:

    * `b_low`: if the robot needs to be recharged
    * `map_OK`: when map construction ends

- `class CHOOSE_DESTINATION(samch.State)`: Class implementing FSM state conserning the choise of the destination.

    Within the execute function, a call is made to the finction `Destination()` and 5 seconds are waited.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    *  `destination`: when the location in which the robot is to move is chosen

    
- `class RANDOM_MOVEMNT(smach.State)`: Class implementing FSM state concerning the random movement.

    Within the execute function, a call is made to the server connecting the FSM to the TopologicalMap node, and 3 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    *  `move`: if the robot can move between the rooms
    
- `class ROOM_E(smach.State)`: Class implementing FSM state concerning the room E.

    Within the execute function, a call is made to the server connecting the FSM to the TopologicalMap node, and 3 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `move`: if the robot can move between the rooms

Within the `main()` function, in addition to the node, the main state machine and the sub-machine are initialised.

### Topological Map node 
brief description
parameters
function

### Random Movement node 
brief description
parameters
function

### Battery node 
brief description
parameters
function

rqt_graph FOTO

UML FOTO --> ogni service ripresenta la stessa struttura di response e request messages sia quando comunica con il nodo della macchina a stati sia quando comunica coi nodi relativi ai comportamenti associati ad ogni stato della FSM

Installation and running
-------------------------------
In a terminal type the following commands:
```bashscript
$ mkdir -p ROS_ws/src
$ cd ROS_ws/src
$ git clone https://github.com/AliceRivi14/Assignment1_ExpRoLab.git
$ cd ..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

To be able to run the nodes you must frun the launch file:
```bashscript
$ roslaunch assignment_1 architecture.launch
```
Oltre al terminale in cui viene lancito il file, appaiono 4 terminali:
* SM
* MAP
* RAND
* BATT

NEL CASO DI PROBLEMI DOVUTI AL CRASHARE DEI NODI, APRIRE 5 TERMINALI, LANCIARE IL roscore E UN NODO PER OGNI TERMINALE NEL SEGUENTE ORDINE
armor
???
sm

Note: Cambiare il percorso dell'Armor_Client nei nodi `‘[ws_path]/assignment_1/ontology/Map.owl’`
