Behavioural Architechture
================================
**A ROS-based assignment for the Experimental Robotics Laboratory course held at the University of Genoa.**

*Author: Alice Rivi <S5135011@studenti.unige.it>*

Introduction 
-----------------

This software architecture has been developed by [Student Robotics](https://studentrobotics.org) in ROS to simulate a surveillance robot.
The whole software is provided in Python 3.

Documentation via [Sphinx](https://www.sphinx-doc.org/en/master/) can be found **[here]**.

The software developed uses a [Smach Finite State Machine](http://wiki.ros.org/smach) FSM and builds an ontology with aRMOR, using the [armor_py_api](https://github.com/EmaroLab/armor_py_api).

The scenario involves a survillance robot operating in a 2D indoor environment, without obstacles, made of 4 rooms (R1, R2, R3, R4) and 3 corridors (E, C1, C2).

![Map](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/Map.png)

The behavior of the robot is divided into 2 phases:

Phase 1:
1. The robot start in the E location;
2. The robot waits until it receives the information to build the topological map;
3. The robot builts the map.

Phase 2:
1. The robot moves through locations following a surveillance policy:
    1. It should mainly stay on corridors,
    2. If a reachable room has not been visited for some times it should visit it;
2. The robot moves in a new location, and waits for some times before to visit another location. 
3. When the robot’s battery is low, the robot goes in the E location, and wait for some times before to start again with the above behavior.


Project Structure
------------------------
There are four nodes in this software architecture:
* StateMachine
* TopologicalMap
* RandomMovement
* Battery

![UML](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/UML.png)

### StateMachine node

In this node, the state machine and the sub-state machine are initialised.

**viewer FOTO**

* Client:

    `/Battery_Switch` to active the ROOM_E state

    `/Movement_Switch` to active the RANDOM_MOVEMENT state

    `/Mapping_Switch` to active the TOPOLOGICAL_MAP state

* Service:

    `/B_Switch` to communicate the need for recharging the battery

There are 4 classes representing the states of the finite state machine. Each state is characterised by the initialisation function `__init__(self)` and the function representing the state execution `execute(self, userdata)`.

- `class TOPOLOGICAL_MAP(smach.State)`: Class implementing FSM state concerning the topological map.

    Within the execute function, a call is made to the server connecting the FSM to the TopologicalMap node, and 3 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:

    * `b_low`: if the robot needs to be recharged
    * `map_OK`: when map construction ends

- `class CHOOSE_DESTINATION(samch.State)`: Class implementing FSM state conserning the choise of the destination.

    Within the execute function, a call is made to the function `Destination()` and 5 seconds are waited.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `destination`: when the location in which the robot is to move is chosen

    
- `class RANDOM_MOVEMNT(smach.State)`: Class implementing FSM state concerning the random movement.

    Within the execute function, a call is made to the server connecting the FSM to the RandomMovement node, and 5 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    *  `move`: if the robot can move between the rooms
    
- `class ROOM_E(smach.State)`: Class implementing FSM state concerning the room E.

    Within the execute function, a call is made to the server connecting the FSM to the Battery node, and 5 seconds are waited for the requested behaviour to be executed.

    Returns the transition of the FSM to be carried out:
    
    * `b_low`: if the robot needs to be recharged
    * `move`: if the robot can move between the rooms
    
Running the FSM node in the terminal should show similar output

![Terminal_StateMachine.png](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/Terminal_StateMachine.png)

### Topological Map node 

This node enables the behaviour associated with the TOPOLOGICAL_MAP state to be performed, simulating the construction of the topological map.

* Client:

    `ArmorClient`

* Service:

    `/Mapping_Switch` to active the TOPOLOGICAL_MAP state

There are 2 functions:

* `LoadMap()`:

    through the aRMOR client, the topology map can be created by loading the ontology. This map was created through the use of [Protégé](https://protege.stanford.edu/) and aved in the file [Map.owl](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/ontology/Map.owl).
    
* `Mapping_Switch(req)`:

    service callback.


### Random Movement node 

This node enables the behaviour associated with the RANDOM_MOVEMENT state to be performed, simulating the robot's movement between locations. 

* Client:

    `ArmorClient`
    
    `MoveBaseAction`

* Service:

    `/Mapping_Switch` to active the RANDOM_MOVEMENT state

    `MoveBaseGoal`

There are 2 functions:

* `MoveBaseA()`:

    provides an implementation of a [MoveBaseAction](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseAction.html) which, given a position goal, will attempt to reach it.

    If the position is not reached within a certain time (3.0 seconds) or if the signal of battery low is sent, the goal is cancelled.
    
* `Movement_Switch(req)`:

    service callback.

Running the RandomMovement node in the terminal should show similar output

![Terminal_RandomMovement.png](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/Terminal_RandomMovement.png)

### Battery node 

This node performs the behaviour associated with the ROOM_E state, simulating the charging of the robot's battery and randomly alerts when the robot needs to be recharged.

* Client:

    `/B_Switch` to communicate the need for recharging the battery
    
    `ArmorClient`

* Service:

    `/Recharging_Switch` to active the RANDOM_MOVEMENT state

There are 1 functions:
    
* `Battery_Switch(req)`:

    service callback.
    
Running the RandomMovement node in the terminal should show similar output

![Terminal_Battery.png](https://github.com/AliceRivi14/Assignment1_ExpRoLab/blob/main/images/Terminal_Battery.png)

In each file, the python script **[Functions.py]** is imported, in which the various functions used by several nodes are defined.

Installation and running
-------------------------------

In a terminal type the following commands:
```bashscript
$ mkdir -p ROS_ws/src/assignment1
$ cd ROS_ws/src/assignment1
$ git clone https://github.com/AliceRivi14/Assignment1_ExpRoLab.git
$ cd ../..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

To be able to run the nodes you must first run the aRMOR service
```bashscript
$ rosrun armor execute it.emarolab.armor.ARMORMainService
```
And the run the launch file:
```bashscript
$ roslaunch assignment1 architecture.launch
```
In addition to the terminal in which the file is launched, 4 terminals relating to the four ROS nodes appear.

In case of problems due to some nodes crashing, run the aRMOR service and open 4 more terminals.
In these terminals, execute the nodes in the following order:
```bashscript
$ rosrun assignment1 TopologicalMap.py
$ rosrun assignment1 RandomMovement.py
$ rosrun assignment1 Battery.py
$ rosrun assignment1 StateMachine.py
```

> Note: Change the Path in the `TopologicalMap script` `‘[ws_path]/assignment_1/ontology/Map.owl’` and the import path in each script.





