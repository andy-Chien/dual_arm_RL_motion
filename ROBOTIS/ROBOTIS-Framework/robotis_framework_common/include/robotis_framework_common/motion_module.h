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
 * motion_module.h
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#ifndef ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_
#define ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_


#include <map>
#include <string>

#include "singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"

namespace robotis_framework
{

enum ControlMode
{
  PositionControl,
  VelocityControl,
  TorqueControl
};

class MotionModule
{
protected:
  bool        enable_;
  std::string module_name_;
  ControlMode control_mode_;

public:
  std::map<std::string, DynamixelState *> result_;

  virtual ~MotionModule() { }

  std::string getModuleName()   { return module_name_; }
  ControlMode getControlMode()  { return control_mode_; }

  void setModuleEnable(bool enable)
  {
    if(this->enable_ == enable)
      return;

    this->enable_ = enable;
    if(enable)
      onModuleEnable();
    else
      onModuleDisable();
  }
  bool getModuleEnable() { return enable_; }

  virtual void  onModuleEnable() { }
  virtual void  onModuleDisable() { }

  virtual void  initialize(const int control_cycle_msec, Robot *robot) = 0;
  virtual void  process(std::map<std::string, Dynamixel *> dxls, std::map<std::string, double> sensors) = 0;

  virtual void	stop() = 0;
  virtual bool	isRunning() = 0;
};


}


#endif /* ROBOTIS_FRAMEWORK_COMMON_MOTION_MODULE_H_ */
