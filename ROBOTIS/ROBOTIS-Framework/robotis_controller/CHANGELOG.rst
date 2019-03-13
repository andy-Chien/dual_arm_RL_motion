^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2018-03-22)
------------------
* added serivce for setting module
* deleted comment for debug
* modified to prevent duplicate indirect address write
* added boost system dependencies
* fixed a bug that occure when handling bulk read item that does not exist.
* Contributors: Kayman, Zerom, Pyo

0.2.8 (2018-03-20)
------------------
* modified CMakeLists.txt for system dependencies (yaml-cpp)
* Contributors: Zerom, Pyo

0.2.7 (2018-03-15)
------------------
* changed the License and package format to version 2
* changed all values read by bulk read are saved to dxl_state\_->bulk_read_table\_.
* Modified to prevent duplicate indirect address write
* Contributors: SCH, Zerom, Pyo

0.2.6 (2017-08-09)
------------------
* multi thread bug fixed.
* unnecessary debug print removed.
* OpenCR control table item name changed. (torque_enable -> dynamixel_power)
* fixed to not update update_time_stamp\_ if bulk read fails.
* Contributors: Zerom

0.2.5 (2017-06-09)
------------------
* updated for yaml-cpp dependencies
* Contributors: SCH

0.2.4 (2017-06-07)
------------------
* added cmake_modules in package.xml
* Contributors: SCH

0.2.3 (2017-05-23)
------------------
* updated the cmake file for ros install
* Contributors: SCH

0.2.2 (2017-04-24)
------------------
* updated robotis_controller.cpp
* changed to read control cycle from .robot file
* Contributors: Zerom

0.2.1 (2016-11-23)
------------------
* Merge the changes and update
* - Direct Control Mode bug fixed.
* update
* - added writeControlTableCallback
* - added WriteControlTable msg callback
* mode change debugging
* - optimized cpu usage by spin loop (by astumpf)
* - robotis_controller process() : processing order changed.
  * 1st : packet communication
  * 2nd : processing modules
* - dependencies fixed. (Pull requests `#26 <https://github.com/ROBOTIS-GIT/ROBOTIS-Framework/issues/26>`_)
* - make setJointCtrlModuleCallback() to the thread function & improved.
* - modified dependency problem.
* - reduce CPU consumption
* Contributors: Jay Song, Pyo, Zerom, SCH

0.2.0 (2016-08-31)
------------------
* bug fixed (position pid gain & velocity pid gain sync write).
* added velocity_to_value_ratio to DXL Pro-H series.
* changed some debug messages.
* added velocity p/i/d gain and position i/d gain sync_write code.
* SyncWriteItem bug fixed.
* add function / modified the code simple (using auto / range based for loop)
* added XM-430-W210 / XM-430-W350 device file.
* rename ControlMode(CurrentControl -> TorqueControl)
* rename (port_to_sync_write_torque\_ -> port_to_sync_write_current\_)
* rename (present_current\_ -> present_torque\_)
* modified torque control code
* fixed typos / changed ROS_INFO -> fprintf (for processing speed)
* startTimer() : after bulkread txpacket(), need some sleep()
* changed the order of processing in the Process() function.
* added missing mutex for gazebo
* fixed crash when running in gazebo simulation
* sync write bug fix.
* added position_p_gain sync write
* MotionModule/SensorModule member variable access changed (public -> protected).
* Contributors: Jay Song, Zerom, Pyo, SCH

0.1.1 (2016-08-18)
------------------
* updated the package information

0.1.0 (2016-08-12)
------------------
* first public release for Kinetic
* modified the package information for release
* develop branch -> master branch
* function name changed : DeviceInit() -> InitDevice()
* Fixed high CPU consumption due to busy waits
* add SensorState
  add Singleton template
* XM-430 / CM-740 device file added.
  Sensor device added.
* added code to support the gazebo simulator
* added first bulk read failure protection code
* renewal
* Contributors: Alexander Stumpf, Jay Song, Zerom, Pyo
