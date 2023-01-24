# Motion controller for Dual-Arm Robot

### [Paper](https://ieeexplore.ieee.org/abstract/document/9345768) | [Video - training](https://youtube.com/shorts/92U7JFCy0Ck?feature=share) | [Video - implement](https://youtu.be/DZbsRPvqpTE)

![implement on real robot](implement.gif)

### Environment
  Ubuntu 16.04
  ROS kinetic
  Tensorflow 1.xx

## Install Packages
```bash
$ sudo apt-get install ros-<distro>-qt-build
$ sudo apt-get install ros-<distro>-rosbridge-server
$ sudo apt-get install ros-<distro>-rosbridge-server
$ sudo apt-get install ros-<distro>-rosserial-python ros-kinetic-rosserial-arduino
# see linear_motion README.md to install libmodbus
```

## Build
```bash
$ cd <work_space>/
$ catkin_make --pkg dual_arm_control
$ catkin_make
```
