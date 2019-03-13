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
 * sensor_state.h
 *
 *  Created on: 2016. 5. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_SENSOR_STATE_H_
#define ROBOTIS_DEVICE_SENSOR_STATE_H_


#include "time_stamp.h"

namespace robotis_framework
{

class SensorState
{
public:
  TimeStamp update_time_stamp_;

  std::map<std::string, uint32_t> bulk_read_table_;

  SensorState()
    : update_time_stamp_(0, 0)
  {
    bulk_read_table_.clear();
  }
};

}


#endif /* ROBOTIS_DEVICE_SENSOR_STATE_H_ */
