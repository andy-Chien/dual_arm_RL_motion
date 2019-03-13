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
 * dynamixel.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_H_


#include <map>
#include <vector>
#include <string>

#include "control_table_item.h"
#include "device.h"
#include "dynamixel_state.h"

namespace robotis_framework
{

class Dynamixel : public Device
{
public:
  std::string     ctrl_module_name_;
  DynamixelState *dxl_state_;

  double  velocity_to_value_ratio_;
  double  torque_to_current_value_ratio_;

  int32_t value_of_0_radian_position_;
  int32_t value_of_min_radian_position_;
  int32_t value_of_max_radian_position_;
  double  min_radian_;
  double  max_radian_;

  ControlTableItem *torque_enable_item_;
  ControlTableItem *present_position_item_;
  ControlTableItem *present_velocity_item_;
  ControlTableItem *present_current_item_;
  ControlTableItem *goal_position_item_;
  ControlTableItem *goal_velocity_item_;
  ControlTableItem *goal_current_item_;
  ControlTableItem *position_p_gain_item_;
  ControlTableItem *position_i_gain_item_;
  ControlTableItem *position_d_gain_item_;
  ControlTableItem *velocity_p_gain_item_;
  ControlTableItem *velocity_i_gain_item_;
  ControlTableItem *velocity_d_gain_item_;

  Dynamixel(int id, std::string model_name, float protocol_version);

  double  convertValue2Radian(int32_t value);
  int32_t convertRadian2Value(double radian);

  double  convertValue2Velocity(int32_t value);
  int32_t convertVelocity2Value(double velocity);

  double  convertValue2Torque(int16_t value);
  int16_t convertTorque2Value(double torque);
};

}


#endif /* ROBOTIS_DEVICE_DYNAMIXEL_H_ */
