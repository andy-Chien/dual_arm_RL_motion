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
 * device.h
 *
 *  Created on: 2016. 5. 12.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DEVICE_H_
#define ROBOTIS_DEVICE_DEVICE_H_


#include <map>
#include <string>
#include <vector>

#include "control_table_item.h"

namespace robotis_framework
{

class Device
{
public:
  uint8_t     id_;
  float       protocol_version_;
  std::string model_name_;
  std::string port_name_;

  std::map<std::string, ControlTableItem *> ctrl_table_;
  std::vector<ControlTableItem *>           bulk_read_items_;

  virtual ~Device() { }
};

}


#endif /* ROBOTIS_DEVICE_DEVICE_H_ */
