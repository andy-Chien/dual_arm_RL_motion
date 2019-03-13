#!/bin/bash
#echo aa | sudo -S chmod 777 /dev/ttyUSB0
#echo aa | sudo -S chmod 777 /dev/ttyUSB1
#echo Launch manipulator
#roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=true &
#echo Run modbus

rosrun linear_motion LM_Control.py 1 $1 & 
sleep 0.3
rosrun linear_motion LM_Control.py 2 $1
