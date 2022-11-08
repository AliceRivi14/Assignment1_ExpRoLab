# Assignment 1 

Behavioural Architechture
================================

Descrivi lo scenario e il robot, copia l'intro dell'assignment = breve intro

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
$ roslaunch assignment_1 architecture
```
Oltre al terminale in cui viene lancito il file, appaiono 4 terminali:
* SM
* MAP
* RAND
* BATT

Note: Cambiare il percorso dell'Armor_Client nei nodi `‘[ws_path]/assignment_1/ontology/Map.owl’`
