# wrs2018
TKU ICLab join 2018 WRS in Tokoyo

youjun well done!!

```bash
$ cd <timda_ws>/src/
$ source devel/setup.bash
```

# TIMDA IBM Assistant Web UI

## Necessary Packages
```bash
$ sudo apt-get install ros-<distro>-qt-build
$ sudo apt-get install ros-<distro>-rosbridge-server
$ sudo apt-get install ros-<distro>-rosbridge-server
$ sudo apt-get install ros-kinetic-rosserial-python ros-kinetic-rosserial-arduino
# see linear_motion README.md to install libmodbus
# install nvm, nodejs, npm
$ git clone git://github.com/creationix/nvm.git ~/.nvm
$ echo ". ~/.nvm/nvm.sh" >> ~/.bashrc
$ source ~/.bashrc
$ nvm install 8.12.0
$ nvm alias default 8.12.0
$ curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.33.2/install.sh | bash 
```

## Build-up
```bash
$ cd <timda_ws>/
$ catkin_make --pkg dual_arm_control
$ catkin_make
```
### Installization
```bash
$ cd <timda_ws>/src/web/myapp
$ npm install
```
### Run Web Server
```bash
$ node index.js
```
### Run Assistant Strategy
```bash
$ rosrun strategy custom_server.py
```
### Rotate Touch Screen
```bash
$ cd <timda_ws>/src/nodejs_pkg/web/script
$ ./rotate_desktop.sh [normal|inverted|right|left] [revert_seconds]
```