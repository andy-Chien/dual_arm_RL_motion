^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_device
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2018-03-22)
------------------
* modified to prevent duplicate indirect address write
* fixed a bug that occure when handling bulk read item that does not exist
* Contributors: Zerom

0.2.8 (2018-03-20)
------------------
* added RH-P12-RN.device file
* Contributors: Zerom, Pyo

0.2.7 (2018-03-15)
------------------
* fixed a bug that occur when handling bulk read item that does not exist
* changed the License and package format to version 2
* Contributors: SCH, Pyo 

0.2.6 (2017-08-09)
------------------
* OpenCR control table item name changed. (torque_enable -> dynamixel_power)
* fixed to not update update_time_stamp\_ if bulk read fails.
* Contributors: Zerom

0.2.5 (2017-06-09)
------------------
* none

0.2.4 (2017-06-07)
------------------
* none

0.2.3 (2017-05-23)
------------------
* updated the cmake file for ros install
* Contributors: SCH

0.2.2 (2017-04-24)
------------------
* added a deivce: OpenCR
* changed to read control cycle from .robot file
* Contributors: Zerom, Kayman

0.2.1 (2016-11-23)
------------------
* Merge the changes and update
* mode change debugging
* - convertRadian2Value / convertValue2Radian : commented out the code that limits the maximum/minimum value.
* - modified dependency problem.
* Contributors: Jay Song, Pyo, Zerom, SCH

0.2.0 (2016-08-31)
------------------
* bug fixed (position pid gain & velocity pid gain sync write).
* added velocity_to_value_ratio to DXL Pro-H series.
* added velocity p/i/d gain and position i/d gain sync_write code.
* fixed robotis_device build_depend.
* added XM-430-W210 / XM-430-W350 device file.
* rename (present_current\_ -> present_torque\_)
* modified torque control code
* added device file for MX-64 / MX-106
* adjusted position min/max value. (MX-28, XM-430)
* Contributors: Zerom, Pyo

0.1.1 (2016-08-18)
------------------
* updated the package information
* Contributors: Zerom

0.1.0 (2016-08-12)
------------------
* first public release for Kinetic
* modified the package information for release
* develop branch -> master branch
* Setting the license to BSD.
* add SensorState
  add Singleton template
* XM-430 / CM-740 device file added.
  Sensor device added.
* modified.
* variable name changed.
  ConvertRadian2Value / ConvertValue2Radian function bug fixed.
* added code to support the gazebo simulator
* renewal
* Contributors: Zerom
