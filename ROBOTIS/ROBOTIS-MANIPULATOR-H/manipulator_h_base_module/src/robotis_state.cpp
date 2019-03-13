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
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: SCH
 */

#include "manipulator_h_base_module/robotis_state.h"

using namespace robotis_manipulator_h;

RobotisState::RobotisState()
{
  is_moving_ = false;
  is_ik = false;

  cnt_      = 0;
  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);
  calc_task_tra_  = Eigen::MatrixXd::Zero(all_time_steps_, 3);
  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, 1);

  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  // for inverse kinematics;
  ik_solve_ = false;

  ik_target_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

  ik_start_rotation_  = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  ik_target_rotation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);

  ik_start_phi_ = 0;
  ik_target_phi_ = 0;
  slide_pos_    = 0;

  ik_id_start_  = 0;
  ik_id_end_    = 0;
}

RobotisState::~RobotisState()
{
}

void RobotisState::setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation, double start_phi)
{
  for (int dim = 0; dim < 3; dim++)
    ik_target_position_.coeffRef(dim, 0) = calc_task_tra_.coeff(cnt, dim);
  
  // Eigen::Vector3d start_euler = ManipulatorKinematicsDynamics::rotation2rpy(start_rotation);
  // start_euler(0) = (start_euler(0)+M_PI)/2;
  // start_rotation = robotis_framework::convertRPYToRotation(start_euler(0), start_euler(1),start_euler(2));

  Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

  Eigen::Quaterniond target_quaternion(kinematics_pose_msg_.pose.orientation.w,
                                       kinematics_pose_msg_.pose.orientation.x,
                                       kinematics_pose_msg_.pose.orientation.y,
                                       kinematics_pose_msg_.pose.orientation.z);

  // Eigen::Matrix3d target_rotation = robotis_framework::convertQuaternionToRotation(target_quaternion);                                    
  // Eigen::Vector3d target_euler = ManipulatorKinematicsDynamics::rotation2rpy(target_rotation);
  // Eigen::Vector3d m_pi(M_PI, M_PI, M_PI);
  // target_euler = (target_euler+m_pi)/2;
  // target_rotation = robotis_framework::convertRPYToRotation(target_euler(0), target_euler(1),target_euler(2));
  // target_quaternion = target_rotation;

  double count = (double) cnt / (double) all_time_steps_;

  Eigen::Quaterniond quaternion = start_quaternion.slerp(count, target_quaternion);
  // std::cout<<"target_quaternion : "<<quaternion.w()<<"  "<<quaternion.x()<<"  "<<quaternion.y()<<"  "<<quaternion.z()<<"  "<<std::endl;

  ik_target_phi_ = start_phi + count * (kinematics_pose_msg_.phi - start_phi);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
  // target_euler = ManipulatorKinematicsDynamics::rotation2rpy(ik_target_rotation_);
  // target_euler = target_euler*2 - m_pi;
  // ik_target_rotation_ = robotis_framework::convertRPYToRotation(target_euler(0), target_euler(1),target_euler(2));

  // std::cout<<"ik_target_rotation_ik_target_rotation_"<<std::endl<<ik_target_rotation_<<std::endl;
}

