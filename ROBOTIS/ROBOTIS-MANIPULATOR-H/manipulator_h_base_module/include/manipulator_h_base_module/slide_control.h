/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * DemoModule.h
 *
 *  Created on: 2016. 3. 9.
 *      Author: SCH
 */

#ifndef SLIDE_CONTROL
#define SLIDE_CONTROL

#include <map>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "linear_motion/Slide_Feedback.h"
#include "manipulator_h_base_module_msgs/SlideCommand.h"   //new


namespace robotis_manipulator_h
{
class slide_control
{
  ros::NodeHandle n;
  ros::Publisher  slide_pos_pub; 
  ros::Publisher  slide_cmd_pub;

public:
  ros::Subscriber slide_fdb_sub;
  
  slide_control();
  ~slide_control();

  /* ROS Topic Callback Functions */

  void slide_pub ();
  void slideFeedback(const linear_motion::Slide_Feedback::ConstPtr& msg);

  double goal_slide_pos;
  double result_slide_pos;
  double slide_pos;
  bool   gazebo_mode;
  bool   is_busy;

};

}

#endif /* MANIPULATOR_BASE_MODULE_BASE_MODULE_H_ */
