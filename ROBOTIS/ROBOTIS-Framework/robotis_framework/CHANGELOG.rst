^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotis_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2018-03-22)
------------------
* added serivce for setting module
* deleted comment for debug
* modified to prevent duplicate indirect address write
* added boost system dependencies
* fixed a bug that occure when handling bulk read item that does not exist
* Contributors: Kayman, Zerom, Pyo

0.2.8 (2018-03-20)
------------------
* added RH-P12-RN.device file
* modified CMakeLists.txt for system dependencies (yaml-cpp)
* Contributors: Zerom, Pyo

0.2.7 (2018-03-15)
------------------
* changed all values read by bulk read are saved to dxl_state\_->bulk_read_table\_.
* Modified to prevent duplicate indirect address write
* fixed a bug that occur when handling bulk read item that does not exist
* changed the License and package format to version 2
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
* updated for yaml-cpp dependencies (robotis_controller)
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
* added a deivce: OpenCR
* updated robotis_controller.cpp
* changed to read control cycle from .robot file
* Contributors: Zerom, Kayman

0.2.1 (2016-11-23)
------------------
* Merge the changes and update

0.2.0 (2016-08-31)
------------------
* updated CHANGLOG.rst for minor release

0.1.1 (2016-08-18)
------------------
* updated the package information

0.1.0 (2016-08-12)
------------------
* make a meta-package
