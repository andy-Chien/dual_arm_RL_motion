# Vacuum Gripper
ROS communicates with arduino board for controlling gripper.


## How to Control Gripper
### Run serial_node to Communicate
1. Connect arduino mega board to computer.
2. Revise permission of port of arduino board.
```bash
sudo chmod a+rw /dev/$SOMEDEVICE
```
3. Run roscore
4. Run serial_node.
```bash
# rosrun rosserial_python serial_node.py <port of device>
rosrun rosserial_python serial_node.py /dev/$SOMEDEVICE
```

### Calibration Gripper
* Use rosservice
```bash
# rosservice call <service name> <request>
# 1. disable motor torque
rosservice call /right/suction_cmd "cmd: 'calibration'"

# 2. set maximum position of motor
rosservice call /right/suction_cmd "cmd: 'setMaxPos'"

# 3. set minimum position of motor
rosservice call /right/suction_cmd "cmd: 'setMinPos'"
```

### Control Gripper
* Use rosservice
```bash
# rosservice call <service name> <request>
# 1. control motor to turn clockwise
rosservice call /right/suction_cmd "cmd: 'suctionUp'"

# 2. control motor to turn counterclockwise
rosservice call /right/suction_cmd "cmd: 'suctionDown'"

# 3. control gripper to suck
rosservice call /right/suction_cmd "cmd: 'vacuumOn'"

# 4. release gripper
rosservice call /right/suction_cmd "cmd: 'vacuumOff'"

# 5. control motor to turn specified angle (0 ~ -90)
rosservice call /right/suction_cmd "cmd: '-45.0'"
```
* Use python api <br>
  reference to [arm_control/suction.py](../arm_control/src/arm_control/suction.py)

### Get Gripper Feedback
* Use rostopic
```bash
# rostopic echo <gripper state topic>
rostopic echo /right/is_grip
```
* Use python api <br>
  reference to [arm_control/suction.py](../arm_control/src/arm_control/suction.py)


## Request Package of ROS
* Install rosserial_python and rosserial_arduino package
```bash
sudo apt-get install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino
```


## Generate Arduino Libraries of ROS
1. Source workspace of custom messages
```bash
source $SOMEWORKSPACE/devel/setup.bash
```
2. Generate arduino libraries <br>
   the folder name of arduino libraries is "ros_lib" was generated in current directory.
```bash
# rosrun rosserial_arduino make_libraries.py <directory of generation libraries>
rosrun rosserial_arduino make_libraries.py .
```
3. Move "ros_lib" to "Arduino/libraries"
```bash
mv ros_lib $HOME/Arduino/libraries
```
* Copy "DynamixelSerial1" to "Arduino/libraries" <br>
  the library is used to control dynamixel motor.
```bash
cp -r $SOMEWHERE/vacuum/arduino_ros_vacuumServiceServer/DynamixelSerial1 $HOME/Arduino/libraries
```


## Upload Arduino Program
1. Open arduino application.
2. Open "arduino_ros_vacuumServiceServer.ino" in arduino app.
3. Connect arduino mega board to computer.
4. Revise permission of port of arduino board.
5. Set infomation of board and port in arduino app. <br>
<img src="./pictures/arduino_toolmenu.png" width="40%" height="40%">
6. Upload the program using arduino app.
