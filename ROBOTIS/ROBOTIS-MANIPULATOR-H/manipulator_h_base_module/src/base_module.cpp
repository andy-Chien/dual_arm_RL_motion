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
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: Zerom
 */

#include <stdio.h>

#include "manipulator_h_base_module/base_module.h"

using namespace robotis_manipulator_h;
bool is_train = false;

BaseModule::BaseModule()
  : control_cycle_msec_(0)
{
  stop_flag     = false;
  wait_flag     = false;
  enable_       = false;
  module_name_  = "base_module";
  control_mode_ = robotis_framework::PositionControl;

  tra_gene_thread_ = NULL;
/*
  result_["joint1"] = new robotis_framework::DynamixelState();
  result_["joint2"] = new robotis_framework::DynamixelState();
  result_["joint3"] = new robotis_framework::DynamixelState();
  result_["joint4"] = new robotis_framework::DynamixelState();
  result_["joint5"] = new robotis_framework::DynamixelState();
  result_["joint6"] = new robotis_framework::DynamixelState();

  joint_name_to_id_["joint1"] = 1;
  joint_name_to_id_["joint2"] = 2;
  joint_name_to_id_["joint3"] = 3;
  joint_name_to_id_["joint4"] = 4;
  joint_name_to_id_["joint5"] = 5;
  joint_name_to_id_["joint6"] = 6;
*/
  robotis_      = new RobotisState();
  joint_state_  = new BaseJointState();
  manipulator_  = new ManipulatorKinematicsDynamics(ARM);
  slide_        = new slide_control();
}

BaseModule::~BaseModule()
{
  queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;
  int id=1;
  for (auto& it : robot->dxls_)
  {
    result_[it.first] = new robotis_framework::DynamixelState();
    joint_name_to_id_[it.first] = id++;
  }
  queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));
}

void BaseModule::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse movement time
  double _mov_time;
  _mov_time = doc["mov_time"].as<double>();

  robotis_->mov_time_ = _mov_time;

  // parse target pose
  YAML::Node _tar_pose_node = doc["tar_pose"];
  for (YAML::iterator _it = _tar_pose_node.begin(); _it != _tar_pose_node.end(); ++_it)
  {
    int _id;
    double _value;

    _id = _it->first.as<int>();
    _value = _it->second.as<double>();

    robotis_->joint_ini_pose_.coeffRef(_id, 0) = _value * DEGREE2RADIAN;
  }

  robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
  robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);
}

void BaseModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  /* subscribe topics */
  ros::Subscriber stop_sub = ros_node.subscribe("/robot/is_stop", 5,
                                                &BaseModule::stopMsgCallback, this);
  ros::Subscriber wait_sub = ros_node.subscribe("/robot/wait", 5,
                                                &BaseModule::waitMsgCallback, this);
  ros::Subscriber clear_cmd_sub = ros_node.subscribe("/robot/clear_cmd",5,
                                                     &BaseModule::clearCmdCallback, this);
  ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose_msg", 5,
                                                        &BaseModule::initPoseMsgCallback, this);
  ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/base/set_mode_msg", 5,
                                                        &BaseModule::setModeMsgCallback, this);

  ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/base/joint_pose_msg", 5,
                                                          &BaseModule::jointPoseMsgCallback, this);
  ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5,
                                                               &BaseModule::kinematicsPoseMsgCallback, this);
  ros::Subscriber p2p_pose_msg_sub = ros_node.subscribe("/robotis/base/p2p_pose_msg", 5,
                                                               &BaseModule::p2pPoseMsgCallback, this);
  slide_->slide_fdb_sub = ros_node.subscribe("/slide_feedback_msg", 10, &slide_control::slideFeedback, slide_);

  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/base/get_joint_pose",
                                                                       &BaseModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/base/get_kinematics_pose",
                                                                            &BaseModule::getKinematicsPoseCallback, this);
  ros::ServiceServer train_server = ros_node.advertiseService("/train_env",
                                                              &BaseModule::training_callback, this);
  ros::ServiceServer env_reset_server = ros_node.advertiseService("/env_reset",
                                                              &BaseModule::env_reset_callback, this);


  while (ros_node.ok())
  {
    callback_queue.callAvailable();
    usleep(1000);
  }
}

bool BaseModule::env_reset_callback(train::environment::Request &req,
                                   train::environment::Response &res)
{
  double dis, mu;
  is_train = true;
  bool ik_success = false;
  if(req.action.size() > 1)
  {
    Eigen::Vector3d p2p_positoin;
    Eigen::Matrix3d p2p_rotation;
    double p2p_phi;
    
    robotis_->is_ik = true;
    for (int i=1; i<=MAX_JOINT_ID; i++)
    {
      dis = manipulator_->manipulator_link_data_[i]->train_limit_max_ - manipulator_->manipulator_link_data_[i]->train_limit_min_;
      mu = (manipulator_->manipulator_link_data_[i]->train_limit_max_ + manipulator_->manipulator_link_data_[i]->train_limit_min_)/2;
      req.action[i] = req.action[i] * fabs(dis) + (mu - req.action[8]*fabs(dis)/2);
    }
    manipulator_->manipulator_link_data_[0]->slide_position_ = req.action[0];
    for (int i=1; i<=MAX_JOINT_ID; i++)
      manipulator_->manipulator_link_data_[i]->joint_angle_ = req.action[i];
    manipulator_->forwardKinematics(7);
    p2p_positoin = manipulator_->manipulator_link_data_[6]->position_;
    p2p_positoin(2) -= 0.24;
    p2p_rotation<<1,0,0,0,-1,0,0,0,-1;
    p2p_phi = 0;
    slide_->goal_slide_pos = 0;
    robotis_->ik_id_end_   = END_LINK;
    ik_success = manipulator_->inverseKinematics(robotis_->ik_id_end_,
                                                              p2p_positoin, p2p_rotation, p2p_phi, slide_->goal_slide_pos, true);
    robotis_->is_ik = false;
    if(enable_ && ik_success)
    {
      robotis_->all_time_steps_ = 1;
      robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);
      for (int id = 1; id <= MAX_JOINT_ID; id++)
      {
        robotis_->calc_joint_tra_(0, id) = manipulator_->manipulator_link_data_[id]->joint_angle_;
      }
      slide_->goal_slide_pos = 0;
      robotis_->calc_slide_tra_(0, 0) = 0;
      robotis_->cnt_ = 0;
      robotis_->is_moving_ = true;
    }
  }
  else
  {
    ik_success = true;
    manipulator_->forwardKinematics(7);
  }
  
  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(manipulator_->manipulator_link_data_[END_LINK]->orientation_);
  res.state.resize(8);
  res.joint_pos.resize(12);
  res.joint_angle.resize(2);
  res.limit.resize(1);
  // res.success = ik_success;
  // if(!ik_success)
  //   return true;
  res.state[0]= manipulator_->manipulator_link_data_[8]->position_(0);
  res.state[1]= manipulator_->manipulator_link_data_[8]->position_(1);
  res.state[2]= manipulator_->manipulator_link_data_[8]->position_(2);
  res.state[3]= quaternion.w();
  res.state[4]= quaternion.x();
  res.state[5]= quaternion.y();
  res.state[6]= quaternion.z();
  res.state[7]= 2*manipulator_->manipulator_link_data_[END_LINK]->phi_ / M_PI;
  res.joint_pos[0]  = manipulator_->manipulator_link_data_[0]->position_(0);
  res.joint_pos[1]  = manipulator_->manipulator_link_data_[0]->position_(1);
  res.joint_pos[2]  = manipulator_->manipulator_link_data_[0]->position_(2);
  res.joint_pos[3]  = manipulator_->manipulator_link_data_[2]->position_(0);
  res.joint_pos[4]  = manipulator_->manipulator_link_data_[2]->position_(1);
  res.joint_pos[5]  = manipulator_->manipulator_link_data_[2]->position_(2);
  res.joint_pos[6]  = manipulator_->manipulator_link_data_[4]->position_(0);
  res.joint_pos[7]  = manipulator_->manipulator_link_data_[4]->position_(1);
  res.joint_pos[8]  = manipulator_->manipulator_link_data_[4]->position_(2);
  res.joint_pos[9]  = manipulator_->manipulator_link_data_[6]->position_(0);
  res.joint_pos[10] = manipulator_->manipulator_link_data_[6]->position_(1);
  res.joint_pos[11] = manipulator_->manipulator_link_data_[6]->position_(2);
  // res.joint_angle[0] = manipulator_->manipulator_link_data_[0]->slide_position_;
  for (int i=2; i<=MAX_JOINT_ID; i+=4)
  {
    dis = manipulator_->manipulator_link_data_[i]->joint_limit_max_ - manipulator_->manipulator_link_data_[i]->joint_limit_min_;
    res.joint_angle[(i-2)/4] = pow((2*(manipulator_->manipulator_link_data_[i]->joint_angle_ - manipulator_->manipulator_link_data_[i]->joint_limit_min_)/fabs(dis))-1, 3);
  }
  Eigen::Vector3d limit_vec = manipulator_->manipulator_link_data_[6]->position_ - manipulator_->manipulator_link_data_[2]->position_;
  // Eigen::Vector3d vecO;
  // vecO << res.joint_pos[9], res.joint_pos[10], 0;
  double limit_dis = limit_vec.norm();
  // double limit_cos = vecO.dot(limit_vec)/(vecO.norm()*limit_dis);
  limit_dis = ((limit_dis - 0.1)/0.448)*2 - 1;
  res.limit[0] = pow(limit_dis, 3);
  // res.limit[1] = pow(limit_cos, 3);
  res.success = ik_success;
  return true;
}

bool BaseModule::training_callback(train::environment::Request &req,
                                   train::environment::Response &res)
{
  bool limit_success = false;
  bool ik_success = false;
  Eigen::Matrix3d p2p_rotation;
  if(req.action.size() > 1)
  {
    robotis_->ik_id_start_ = 0;
    robotis_->ik_id_end_   = END_LINK;
    
    Eigen::Vector3d p2p_positoin;

    int     max_iter    = 30;
    double  ik_tol      = 1e-3;

    p2p_positoin << req.action[0], 
                    req.action[1], 
                    req.action[2];

    Eigen::Quaterniond p2p_quaterniond(req.action[3],
                                       req.action[4],
                                       req.action[5],
                                       req.action[6]);

    double p2p_phi = req.action[7]*M_PI/2;
    p2p_rotation = robotis_framework::convertQuaternionToRotation(p2p_quaterniond);
    robotis_->is_ik = true;
    // manipulator_->forwardKinematics(7);
    // slide_success = manipulator_->slideInverseKinematics(p2p_positoin, p2p_rotation, 
    //                                                           slide_->slide_pos, slide_->goal_slide_pos);
    // std::cout<<"<<<<<<<<<<<<<<<<<<<slide_->goal_slide_pos<<<<<<<<<<<<<<<<<"<<std::endl<<slide_->goal_slide_pos<<std::endl;
    slide_->goal_slide_pos = 0;
    limit_success = manipulator_->limit_check(p2p_positoin, p2p_rotation);
    if(limit_success)
      ik_success = manipulator_->inverseKinematics(robotis_->ik_id_end_,
                                                              p2p_positoin, p2p_rotation, p2p_phi, slide_->goal_slide_pos, true);
    robotis_->is_ik = false;
  }
  
  if ((ik_success == true && limit_success == true))
  {
    res.success = true;
    if(enable_)
    {
      robotis_->all_time_steps_ = 1;
      robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);
      for (int id = 1; id <= MAX_JOINT_ID; id++)
      {
        robotis_->calc_joint_tra_(0, id) = manipulator_->manipulator_link_data_[id]->joint_angle_;
      }
      slide_->goal_slide_pos = 0;
      robotis_->calc_slide_tra_(0, 0) = 0;
      robotis_->cnt_ = 0;
      robotis_->is_moving_ = true;
    }
  }
  else
  {
    res.success = false;
  }
  double dis;
  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(manipulator_->manipulator_link_data_[END_LINK]->orientation_);
  res.state.resize(8);
  res.joint_pos.resize(12);
  res.joint_angle.resize(2);
  res.limit.resize(1);
  res.state[0]= manipulator_->manipulator_link_data_[8]->position_(0);
  res.state[1]= manipulator_->manipulator_link_data_[8]->position_(1);
  res.state[2]= manipulator_->manipulator_link_data_[8]->position_(2);
  res.state[3]= quaternion.w();
  res.state[4]= quaternion.x();
  res.state[5]= quaternion.y();
  res.state[6]= quaternion.z();
  res.state[7]= 2*manipulator_->manipulator_link_data_[END_LINK]->phi_ / M_PI;
  res.joint_pos[0]  = manipulator_->manipulator_link_data_[0]->position_(0);
  res.joint_pos[1]  = manipulator_->manipulator_link_data_[0]->position_(1);
  res.joint_pos[2]  = manipulator_->manipulator_link_data_[0]->position_(2);
  res.joint_pos[3]  = manipulator_->manipulator_link_data_[2]->position_(0);
  res.joint_pos[4]  = manipulator_->manipulator_link_data_[2]->position_(1);
  res.joint_pos[5]  = manipulator_->manipulator_link_data_[2]->position_(2);
  res.joint_pos[6]  = manipulator_->manipulator_link_data_[4]->position_(0);
  res.joint_pos[7]  = manipulator_->manipulator_link_data_[4]->position_(1);
  res.joint_pos[8]  = manipulator_->manipulator_link_data_[4]->position_(2);
  res.joint_pos[9]  = manipulator_->manipulator_link_data_[6]->position_(0);
  res.joint_pos[10] = manipulator_->manipulator_link_data_[6]->position_(1);
  res.joint_pos[11] = manipulator_->manipulator_link_data_[6]->position_(2);
  // res.joint_angle[0] = manipulator_->manipulator_link_data_[0]->slide_position_;
      
  for (int i=2; i<=MAX_JOINT_ID; i+=4)
  {
    dis = manipulator_->manipulator_link_data_[i]->joint_limit_max_ - manipulator_->manipulator_link_data_[i]->joint_limit_min_;
    res.joint_angle[(i-2)/4] = pow((2*(manipulator_->manipulator_link_data_[i]->joint_angle_ - manipulator_->manipulator_link_data_[i]->joint_limit_min_)/fabs(dis))-1, 3);
  }
  Eigen::Vector3d limit_vec = manipulator_->manipulator_link_data_[6]->position_ - manipulator_->manipulator_link_data_[2]->position_;
  // Eigen::Vector3d vecO;
  // vecO << res.joint_pos[9], res.joint_pos[10], 0;
  double limit_dis = limit_vec.norm();
  // double limit_cos = vecO.dot(limit_vec)/(vecO.norm()*limit_dis);
  limit_dis = ((limit_dis - 0.1)/0.448)*2 - 1;
  res.limit[0] = pow(limit_dis, 3);
  // res.limit[1] = pow(2*limit_cos, 3);
  robotis_->is_ik = false;
  return true;
}
void BaseModule::stopMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data)
    stop();
  else
    stop_flag = false;
}
void BaseModule::waitMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  wait_flag = msg->data;
}
void BaseModule::clearCmdCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data)
  {
    robotis_->is_moving_ = false;
    robotis_->ik_solve_ = false;
    robotis_->cnt_ = 0;
    ROS_INFO("[end] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");
  }
}
void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (robotis_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // parse initial pose
      std::string ini_pose_path = ros::package::getPath("manipulator_h_base_module") + "/config/ini_pose.yaml";
      parseIniPoseData(ini_pose_path);

      tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateInitPoseTrajProcess, this));
      delete tra_gene_thread_;
    }
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

void BaseModule::setModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "stop")
    stop();
  else
  {
    std_msgs::String str_msg;
    str_msg.data = "base_module";

    set_ctrl_module_pub_.publish(str_msg);
  }
  return;
}

bool BaseModule::getJointPoseCallback(manipulator_h_base_module_msgs::GetJointPose::Request &req,
                                      manipulator_h_base_module_msgs::GetJointPose::Response &res)
{
  if (enable_ == false)
    return false;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    for (int name_index = 0; name_index < req.joint_name.size(); name_index++)
    {
      if (manipulator_->manipulator_link_data_[id]->name_ == req.joint_name[name_index])
      {
        res.joint_name.push_back(manipulator_->manipulator_link_data_[id]->name_);
        res.joint_value.push_back(joint_state_->goal_joint_state_[id].position_);

        break;
      }
    }
  }
  res.slide_pos = slide_->slide_pos;

  return true;
}

bool BaseModule::getKinematicsPoseCallback(manipulator_h_base_module_msgs::GetKinematicsPose::Request &req,
                                           manipulator_h_base_module_msgs::GetKinematicsPose::Response &res)
{
  if (enable_ == false)
    return false;

  res.group_pose.position.x = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(0, 0);
  res.group_pose.position.y = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(1, 0);
  res.group_pose.position.z = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(2, 0);
  res.phi =  manipulator_->manipulator_link_data_[END_LINK]->phi_;
  res.orientation.resize(16);
  for(int i=0; i<=3; i++)
  {
    for(int j=0; j<=3; j++)
    {
      res.orientation[i*4 + j] = manipulator_->manipulator_link_data_[END_LINK]->transformation_(i,j);
    }
  }
  Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(manipulator_->manipulator_link_data_[END_LINK]->orientation_);
  // for (int id = 1; id <= MAX_JOINT_ID; id++)
    // std::cout<<"manipulator_->manipulator_link_data_[END_LINK]->angle"<<std::endl<<manipulator_->manipulator_link_data_[id]->joint_angle_<<std::endl;

  // std::cout<<"manipulator_->manipulator_link_data_[END_LINK]->orientation_"<<std::endl<<manipulator_->manipulator_link_data_[END_LINK]->orientation_<<std::endl;
  res.group_pose.orientation.w = quaternion.w();
  res.group_pose.orientation.x = quaternion.x();
  res.group_pose.orientation.y = quaternion.y();
  res.group_pose.orientation.z = quaternion.z();

  for(int i=0; i<=2; i++)
    res.euler.push_back(manipulator_->manipulator_link_data_[END_LINK]->euler(i));

  // std::cout<<"euler = "<<manipulator_->manipulator_link_data_[END_LINK]->euler *180/M_PI<<std::endl;

  return true;
}

void BaseModule::kinematicsPoseMsgCallback(const manipulator_h_base_module_msgs::KinematicsPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  robotis_->kinematics_pose_msg_ = *msg;

  robotis_->ik_id_start_ = 0;
  robotis_->ik_id_end_   = END_LINK;

  if (robotis_->is_moving_ == false)
  {
    tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateTaskTrajProcess, this));
    delete tra_gene_thread_;
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}
// =======================================================================================================================
void BaseModule::p2pPoseMsgCallback(const manipulator_h_base_module_msgs::P2PPose::ConstPtr& msg)
{
  // std::cout<<"asdfasdfasdf"<<std::endl;
  if (enable_ == false)
    return;

  robotis_->p2p_pose_msg_ = *msg;

  robotis_->ik_id_start_ = 0;
  robotis_->ik_id_end_   = END_LINK;

  Eigen::Vector3d p2p_positoin;
  Eigen::Matrix3d p2p_rotation;

  int     max_iter    = 30;
  double  ik_tol      = 1e-3;

  p2p_positoin << robotis_->p2p_pose_msg_.pose.position.x, 
                  robotis_->p2p_pose_msg_.pose.position.y, 
                  robotis_->p2p_pose_msg_.pose.position.z;

  Eigen::Quaterniond p2p_quaterniond(robotis_->p2p_pose_msg_.pose.orientation.w,
                                        robotis_->p2p_pose_msg_.pose.orientation.x,
                                        robotis_->p2p_pose_msg_.pose.orientation.y,
                                        robotis_->p2p_pose_msg_.pose.orientation.z);

  double p2p_phi = robotis_->p2p_pose_msg_.phi;

  p2p_rotation = robotis_framework::convertQuaternionToRotation(p2p_quaterniond);

  manipulator_->forwardKinematics(7);
  // std::cout<<"p2p_positoinp2p_positoin"<<std::endl<<p2p_positoin<<std::endl;
  // std::cout<<"p2p_rotationp2p_rotation"<<std::endl<<p2p_rotation<<std::endl;

  robotis_->is_ik = true;
  
  bool    slide_success = manipulator_->slideInverseKinematics(p2p_positoin, p2p_rotation, 
                                                            slide_->slide_pos, slide_->goal_slide_pos);

  // std::cout<<"<<<<<<<<<<<<<<<<<<<slide_->goal_slide_pos<<<<<<<<<<<<<<<<<"<<std::endl<<slide_->goal_slide_pos<<std::endl;
  bool    ik_success = manipulator_->inverseKinematics(robotis_->ik_id_end_,
                                                            p2p_positoin, p2p_rotation, p2p_phi, slide_->goal_slide_pos, true);

  if (ik_success == true && slide_success == true)
  {
    manipulator_h_base_module_msgs::JointPose p2p_msg;

    for ( int id = 1; id <= MAX_JOINT_ID; id++ )
    {
      p2p_msg.name.push_back(manipulator_->manipulator_link_data_[id]->name_);
      p2p_msg.value.push_back(manipulator_->manipulator_link_data_[id]->joint_angle_);
    }
    p2p_msg.slide_pos = slide_->goal_slide_pos;
    p2p_msg.speed     = robotis_->p2p_pose_msg_.speed;
    robotis_->joint_pose_msg_ = p2p_msg;

    if (robotis_->is_moving_ == false)
    {
      tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateJointTrajProcess, this));
      delete tra_gene_thread_;
    }
    else
    {
      ROS_INFO("previous task is alive");
    }
  }
  else
  {
    ROS_INFO("[end] send trajectory (ik failed)");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory (p2p IK Failed)");
    return;
  }
  robotis_->is_ik = false;
  return;
}
// =======================================================================================================================

void BaseModule::jointPoseMsgCallback(const manipulator_h_base_module_msgs::JointPose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  robotis_->joint_pose_msg_ = *msg;

  if (robotis_->is_moving_ == false)
  {
    tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateJointTrajProcess, this));
    delete tra_gene_thread_;
  }
  else
  {
    ROS_INFO("previous task is alive");
  }

  return;
}

void BaseModule::generateInitPoseTrajProcess()
{
  if (enable_ == false)
    return;

  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value = robotis_->joint_ini_pose_.coeff(id, 0);

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                robotis_->smp_time_, robotis_->mov_time_);

    robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
  }

  slide_->goal_slide_pos = robotis_->joint_ini_pose_.coeff(0, 0);
  generateSlideTrajProcess();
  robotis_->cnt_ = 0;
  robotis_->is_moving_ = true;

  ROS_INFO("[start] send trajectory");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void BaseModule::generateJointTrajProcess()
{
  if (enable_ == false)
    return;

  /* set movement time */
  double mov_speed = robotis_->joint_pose_msg_.speed;
  double tol = 90 * (mov_speed / 100) * DEGREE2RADIAN; // rad per sec
  double mov_time = 1.5;

  double max_diff, abs_diff, slide_diff;
  slide_diff = fabs(robotis_->joint_pose_msg_.slide_pos - slide_->slide_pos);
  max_diff = slide_diff * 1000 * DEGREE2RADIAN;  //slide 1 cm = motor 10 degree to calculate

  
  for (int name_index = 0; name_index < robotis_->joint_pose_msg_.name.size(); name_index++)
  {
    double ini_value;
    double tar_value;

    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
      if (manipulator_->manipulator_link_data_[id]->name_ == robotis_->joint_pose_msg_.name[name_index])
      {
        ini_value = joint_state_->goal_joint_state_[id].position_;
        tar_value = robotis_->joint_pose_msg_.value[name_index];

        break;
      }
    }
    
    abs_diff = fabs(tar_value - ini_value);

    if (max_diff < abs_diff)
      max_diff = abs_diff;
  }

  robotis_->mov_time_ = max_diff / tol;
  int all_time_steps = int(floor((robotis_->mov_time_ / robotis_->smp_time_) + 1.0));
  robotis_->mov_time_ = double(all_time_steps - 1) * robotis_->smp_time_;

  if (robotis_->mov_time_ < mov_time)
    robotis_->mov_time_ = mov_time;

  robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;

  robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);

  /* calculate joint trajectory */
  for (int id = 1; id <= MAX_JOINT_ID; id++)
  {
    double ini_value = joint_state_->goal_joint_state_[id].position_;
    double tar_value;

    for (int name_index = 0; name_index < robotis_->joint_pose_msg_.name.size(); name_index++)
    {
      if (manipulator_->manipulator_link_data_[id]->name_ == robotis_->joint_pose_msg_.name[name_index])
      {
        tar_value = robotis_->joint_pose_msg_.value[name_index];
        break;
      }
    }

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                robotis_->smp_time_, robotis_->mov_time_);

    robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
  }

  slide_->goal_slide_pos = robotis_->joint_pose_msg_.slide_pos;
  generateSlideTrajProcess();

  robotis_->cnt_ = 0;
  robotis_->is_moving_ = true;

  ROS_INFO("[start] send trajectory");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

void BaseModule::generateTaskTrajProcess()
{
  /* set movement time */
  double mov_speed = robotis_->kinematics_pose_msg_.speed;
  double tol = 0.3 * (mov_speed / 100); // m per sec
  double mov_time = 1.5;
  double slide_diff;

  manipulator_->manipulator_link_data_[0]->mov_speed_ = mov_speed;

  Eigen::Vector3d goal_positoin;
  goal_positoin << robotis_->kinematics_pose_msg_.pose.position.x, 
                   robotis_->kinematics_pose_msg_.pose.position.y, 
                   robotis_->kinematics_pose_msg_.pose.position.z;

  Eigen::Quaterniond goal_quaterniond(robotis_->kinematics_pose_msg_.pose.orientation.w,
                                      robotis_->kinematics_pose_msg_.pose.orientation.x,
                                      robotis_->kinematics_pose_msg_.pose.orientation.y,
                                      robotis_->kinematics_pose_msg_.pose.orientation.z);

  Eigen::Matrix3d goal_rotation = robotis_framework::convertQuaternionToRotation(goal_quaterniond);

  bool ik_success = manipulator_->slideInverseKinematics(goal_positoin, goal_rotation, 
                                                            slide_->slide_pos, slide_->goal_slide_pos);

  slide_diff = fabs(robotis_->joint_pose_msg_.slide_pos - slide_->slide_pos) * 2;  //slide 1 cm = arm 2 cm to calculate

  double diff = sqrt(
                      pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(0, 0)
                          - robotis_->kinematics_pose_msg_.pose.position.x, 2)
                    + pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(1, 0)
                          - robotis_->kinematics_pose_msg_.pose.position.y, 2)
                    + pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(2, 0)
                          - robotis_->kinematics_pose_msg_.pose.position.z, 2)
                );
  diff = (diff < slide_diff) ? slide_diff : diff;

  robotis_->mov_time_ = diff / tol;
  int all_time_steps = int(floor((robotis_->mov_time_ / robotis_->smp_time_) + 1.0));
  robotis_->mov_time_ = double(all_time_steps - 1) * robotis_->smp_time_;

  if (robotis_->mov_time_ < mov_time)
    robotis_->mov_time_ = mov_time;

  robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;

  robotis_->calc_task_tra_.resize(robotis_->all_time_steps_, 3);

  /* calculate trajectory */
  for (int dim = 0; dim < 3; dim++)
  {
    double ini_value = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(dim, 0);
    double tar_value;

    if (dim == 0)
      tar_value = robotis_->kinematics_pose_msg_.pose.position.x;
    else if (dim == 1)
      tar_value = robotis_->kinematics_pose_msg_.pose.position.y;
    else if (dim == 2)
      tar_value = robotis_->kinematics_pose_msg_.pose.position.z;

    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                robotis_->smp_time_, robotis_->mov_time_);

    robotis_->calc_task_tra_.block(0, dim, robotis_->all_time_steps_, 1) = tra;
  }

  if (ik_success == true)
  {
    generateSlideTrajProcess();
    robotis_->cnt_ = 0;
    robotis_->is_moving_ = true;
    robotis_->ik_solve_ = true;

    ROS_INFO("[start] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
  }
  else
  {
    ROS_INFO("[end] send trajectory (ik failed)");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory (IK Failed)");

    robotis_->is_moving_ = false;
    robotis_->ik_solve_ = false;
    robotis_->cnt_ = 0;
  }

}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false || wait_flag == true)
    return;

  /*----- write curr position -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

  /*----- forward kinematics -----*/
  if( robotis_->is_ik == false && is_train == false)
  {
    manipulator_->manipulator_link_data_[0]->slide_position_ = slide_->slide_pos;
    for (int id = 1; id <= MAX_JOINT_ID; id++)
      manipulator_->manipulator_link_data_[id]->joint_angle_ = joint_state_->goal_joint_state_[id].position_;
    
    manipulator_->forwardKinematics(7);  // 0 chang to 7 : how many joint
  }

  //slide_->slide_pos = manipulator_->manipulator_link_data_[0]->slide_position_;
  /* ----- send trajectory ----- */

//    ros::Time time = ros::Time::now();
  if (robotis_->is_moving_ == true && robotis_->cnt_ < robotis_->all_time_steps_)
  {
    if (robotis_->cnt_ == 0)
    {
      robotis_->ik_start_rotation_ = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->orientation_;
      robotis_->ik_start_phi_ = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->phi_;
    }
    if (robotis_->ik_solve_ == true)
    {
      robotis_->setInverseKinematics(robotis_->cnt_, robotis_->ik_start_rotation_, robotis_->ik_start_phi_);

      int     max_iter      = 30;
      double  ik_tol        = 1e-3;
      double  tar_slide_pos = robotis_->calc_slide_tra_(robotis_->cnt_, 0);

      robotis_->is_ik = true;
      
      bool    ik_success  = manipulator_->inverseKinematics(robotis_->ik_id_end_,robotis_->ik_target_position_, 
                                                              robotis_->ik_target_rotation_, robotis_->ik_target_phi_, tar_slide_pos, false);
    
      if (ik_success == true)
      {
        for (int id = 1; id <= MAX_JOINT_ID; id++)
          joint_state_->goal_joint_state_[id].position_ = manipulator_->manipulator_link_data_[id]->joint_angle_;
        slide_->goal_slide_pos = robotis_->calc_slide_tra_(robotis_->cnt_, 0);
        slide_->result_slide_pos = robotis_->calc_slide_tra_(robotis_->cnt_, 0);
        if(manipulator_->manipulator_link_data_[0]->singularity_ && robotis_->cnt_ > 1)
        {
          std::cout<<"====robotis_->cnt_====="<<robotis_->cnt_<<" ";
          robotis_->cnt_--;
          // robotis_->cnt_ = (robotis_->cnt_ > 1) ? (robotis_->cnt_-1) : robotis_->cnt_;
          std::cout<<robotis_->cnt_<<std::endl;
        }
          // std::cout<<"==========================process after ik"<<std::endl;
          // manipulator_->forwardKinematics(7);
          // assert(false);
      }
      else
      {
        ROS_INFO("[end] send trajectory (ik failed)");
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory (IK Failed)");

        robotis_->is_moving_ = false;
        robotis_->ik_solve_ = false;
        robotis_->cnt_ = 0;
      }
      robotis_->is_ik = false;
    }
    else
    {
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        joint_state_->goal_joint_state_[id].position_ = robotis_->calc_joint_tra_(robotis_->cnt_, id);

      slide_->goal_slide_pos = robotis_->calc_slide_tra_(robotis_->cnt_, 0);
      slide_->result_slide_pos = robotis_->calc_slide_tra_(robotis_->cnt_, 0);
    }

    robotis_->cnt_++;
  }

  /*----- set joint data -----*/

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
  }
  slide_->slide_pub();
  /*---------- initialize count number ----------*/

  if (robotis_->cnt_ >= robotis_->all_time_steps_ && robotis_->is_moving_ == true && !slide_->is_busy)
  {
    // ROS_INFO("[end] send trajectory");
    // publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

    // slide_->is_end = true;
    robotis_->is_moving_ = false;
    robotis_->ik_solve_ = false;
    robotis_->cnt_ = 0;
  }
}

void BaseModule::stop()
{
  robotis_->is_moving_ = false;
  robotis_->ik_solve_ = false;
  robotis_->cnt_ = 0;

  manipulator_h_base_module_msgs::JointPose stop_msg;

  for ( int id = 1; id <= MAX_JOINT_ID; id++ )
  {
    stop_msg.name.push_back(manipulator_->manipulator_link_data_[id]->name_);
    stop_msg.value.push_back(joint_state_->goal_joint_state_[id].position_);
  }
  stop_msg.slide_pos = slide_->goal_slide_pos;
  stop_msg.speed     = 10;
  robotis_->joint_pose_msg_ = stop_msg;

  if (!stop_flag)
  {
    robotis_->is_moving_ = false;
    robotis_->ik_solve_  = false;
    robotis_->cnt_ = 0;
    tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateJointTrajProcess, this));
    delete tra_gene_thread_;
    stop_flag = true;
    ROS_INFO("!!!!Stop robot arm!!!!");
  }
  return;
}

bool BaseModule::isRunning()
{
  return robotis_->is_moving_;
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Base";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

void BaseModule::generateSlideTrajProcess()
{
  double ini_slide_value;
  double tar_slide_value;



  ini_slide_value = slide_->slide_pos;
  tar_slide_value = slide_->goal_slide_pos;

  Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_slide_value, 0.0, 0.0, tar_slide_value, 0.0, 0.0,
                                                                robotis_->smp_time_, robotis_->mov_time_);
  robotis_->calc_slide_tra_.resize(robotis_->all_time_steps_, 1);
  robotis_->calc_slide_tra_ = tra;  
}
