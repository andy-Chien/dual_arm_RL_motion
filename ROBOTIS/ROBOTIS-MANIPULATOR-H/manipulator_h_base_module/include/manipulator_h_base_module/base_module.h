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

#ifndef MANIPULATOR_BASE_MODULE_BASE_MODULE_H_
#define MANIPULATOR_BASE_MODULE_BASE_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "manipulator_h_base_module_msgs/JointPose.h"
#include "manipulator_h_base_module_msgs/KinematicsPose.h"
#include "manipulator_h_base_module_msgs/P2PPose.h"   //new

#include "manipulator_h_base_module_msgs/GetJointPose.h"
#include "manipulator_h_base_module_msgs/GetKinematicsPose.h"

#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"
#include "robotis_state.h"
#include "slide_control.h"   //new

#include "robotis_controller/robotis_controller.h"
#include "robotis_device/robot.h"

#include "train/environment.h"

namespace robotis_manipulator_h
{

//using namespace ROBOTIS_DEMO;

class BaseJointData
{
public:
  double position_;
  double velocity_;
  double effort_;

  int p_gain_;
  int i_gain_;
  int d_gain_;
  BaseJointData(): position_(0), velocity_(0), effort_(0), p_gain_(0), i_gain_(0), d_gain_(0) {}
};

class BaseJointState
{
public:
  BaseJointData curr_joint_state_[ MAX_JOINT_ID + 1];
  BaseJointData goal_joint_state_[ MAX_JOINT_ID + 1];
  BaseJointData fake_joint_state_[ MAX_JOINT_ID + 1];
};

class BaseModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<BaseModule>
{
private:
  bool stop_flag;
  bool wait_flag;
  int             control_cycle_msec_;
  boost::thread   queue_thread_;
  boost::thread  *tra_gene_thread_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  set_ctrl_module_pub_;

  std::map<std::string, int> joint_name_to_id_;

  void queueThread();

  void parseIniPoseData(const std::string &path);
  void publishStatusMsg(unsigned int type, std::string msg);

public:
  BaseModule();
  virtual ~BaseModule();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setModeMsgCallback(const std_msgs::String::ConstPtr& msg);
  void waitMsgCallback(const std_msgs::Bool::ConstPtr& msg);
  void stopMsgCallback(const std_msgs::Bool::ConstPtr& msg);
  void clearCmdCallback(const std_msgs::Bool::ConstPtr& msg);
  void jointPoseMsgCallback(const manipulator_h_base_module_msgs::JointPose::ConstPtr& msg);
  void kinematicsPoseMsgCallback(const manipulator_h_base_module_msgs::KinematicsPose::ConstPtr& msg);
  void p2pPoseMsgCallback(const manipulator_h_base_module_msgs::P2PPose::ConstPtr& msg);   //new

  bool training_callback(train::environment::Request &req,
                         train::environment::Response &res);///////////////////////////////////////////////////////
  bool env_reset_callback(train::environment::Request &req,
                          train::environment::Response &res);
  bool getJointPoseCallback(manipulator_h_base_module_msgs::GetJointPose::Request &req,
                            manipulator_h_base_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(manipulator_h_base_module_msgs::GetKinematicsPose::Request &req,
                                 manipulator_h_base_module_msgs::GetKinematicsPose::Response &res);

  /* ROS Calculation Functions */
  void generateInitPoseTrajProcess();
  void generateJointTrajProcess();
  void generateTaskTrajProcess();
  void generateSlideTrajProcess();    //new


  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  /* Parameter */
  BaseJointState                 *joint_state_;
  RobotisState                   *robotis_;
  ManipulatorKinematicsDynamics  *manipulator_;
  slide_control                  *slide_;    //new   
};

}

#endif /* MANIPULATOR_BASE_MODULE_BASE_MODULE_H_ */
