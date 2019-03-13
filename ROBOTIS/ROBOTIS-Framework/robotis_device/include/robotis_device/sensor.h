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
 * sensor.h
 *
 *  Created on: 2015. 12. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_SENSOR_H_
#define ROBOTIS_DEVICE_SENSOR_H_

#include <map>
#include <string>
#include <stdint.h>

#include "device.h"
#include "sensor_state.h"
#include "control_table_item.h"

namespace robotis_framework
{

class Sensor : public Device
{
public:
  SensorState *sensor_state_;

  Sensor(int id, std::string model_name, float protocol_version);
};

}


#endif /* ROBOTIS_DEVICE_SENSOR_H_ */
