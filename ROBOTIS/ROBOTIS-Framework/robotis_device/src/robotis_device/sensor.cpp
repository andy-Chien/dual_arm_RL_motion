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
 * sensor.cpp
 *
 *  Created on: 2016. 5. 11.
 *      Author: zerom
 */

#include "robotis_device/sensor.h"

using namespace robotis_framework;

Sensor::Sensor(int id, std::string model_name, float protocol_version)
{
    this->id_ = id;
    this->model_name_ = model_name;
    this->port_name_ = "";
    this->protocol_version_ = protocol_version;
    ctrl_table_.clear();

    sensor_state_ = new SensorState();

    bulk_read_items_.clear();
}
