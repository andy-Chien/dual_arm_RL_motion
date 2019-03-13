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
 * dynamixel_state.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_

#include <stdint.h>

#include "time_stamp.h"

#define INDIRECT_DATA_1     "indirect_data_1"
#define INDIRECT_ADDRESS_1  "indirect_address_1"

namespace robotis_framework
{

class DynamixelState
{
public:
  TimeStamp update_time_stamp_;

  double    present_position_;
  double    present_velocity_;
  double    present_torque_;
  double    goal_position_;
  double    goal_velocity_;
  double    goal_torque_;
  int       position_p_gain_;
  int       position_i_gain_;
  int       position_d_gain_;
  int       velocity_p_gain_;
  int       velocity_i_gain_;
  int       velocity_d_gain_;

  std::map<std::string, uint32_t> bulk_read_table_;

  double    position_offset_;

  DynamixelState()
    : update_time_stamp_(0, 0),
      present_position_(0.0),
      present_velocity_(0.0),
      present_torque_(0.0),
      goal_position_(0.0),
      goal_velocity_(0.0),
      goal_torque_(0.0),
      position_p_gain_(0),
      position_i_gain_(0),
      position_d_gain_(0),
      velocity_p_gain_(0),
      velocity_i_gain_(0),
      velocity_d_gain_(0),
      position_offset_(0)
  {
    bulk_read_table_.clear();
  }
};

}


#endif /* ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_ */
