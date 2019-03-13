# Strategy

Task file inside

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 
See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
sudo apt-get install ros-kinetic-ros-controllers
```
```
sudo apt-get install ros-kinetic-gazebo-ros-control
```

### Installing

#### A step by step series of examples that tell you how to get a development env running
##### Start manager of manipulator

* If you want to run on real robot:
```
roslaunch manipulator_h_manager dual_arm.launch
```
* If you want to run in gazebo:
```
roslaunch manipulator_h_manager dual_arm.launch en_sim:=true
```

## Running the tests

Explain how to run the automated tests for this system

#### Running sample tasks

##### 1. Reset the position of the object (only in simulation)
```
rosrun strategy object_pos.py
```
##### 2. Run task example

* If you want to run on real robot:
```
rosrun strategy example.py 
```
* If you want to run in gazebo:
```
rosrun strategy example.py True
```
