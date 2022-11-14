# Assignment 1 

Behavioural Architechture
================================

This software architecture has been developed by [Student Robotics](https://studentrobotics.org) in ROS to simulate a surveillance robot.
The whole software is provided in Python 3.

The software developed uses a [Smach State Machine](http://wiki.ros.org/smach) and builds an ontology with aRMOR, using the [armor_py_api](https://github.com/EmaroLab/armor_py_api).

The scenario involves a survillance robot operating in a 2D indoor environment, made of 4 rooms (R1, R2, R3, R4) and 3 corridors (E, C1, C2).

![Immagine 2022-11-14 173457]

The behavior of the robot is divided into 2 phases:

* Phase 1:
1. The robot start in the E location;
2. The robot waits until it receives the information to build the topological map ;
3. The robot builts the map.

* Phase 2:
<ol>
<li>The robot moves through locations following a surveillance policy:</li>
 <ol>
  <li>It should mainly stay on corridors,</li>
  <li>If a reachable room has not been visited for some times it should visit it;</li>
 </ol>
<li>The robot moves in a new location, and waits for some times before to visit another location. This behavior is repeated in a infinite loop;</li>
<li>When the robot’s battery is low, the robot goes in the E location, and wait for some times before to start again with the above behavior.</li>
<ol>



ROS Architecture
------------------------

4 nodi:
* SM
* MAP
* MOV
* BATT

### SM node 
viewer FOTO
* TOP
* MOV
  * `b_low`
  * `move`
* BAT
  * `b_low`
  * `move`

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
