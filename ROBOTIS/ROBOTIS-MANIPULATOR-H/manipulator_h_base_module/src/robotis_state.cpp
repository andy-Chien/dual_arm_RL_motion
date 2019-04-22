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
  is_inv = false;

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
  ik_target_quaternion = robotis_framework::convertRotationToQuaternion(ik_target_rotation_);

  ik_start_phi_ = 0;
  ik_target_phi_ = 0;
  slide_pos_    = 0;

  ik_id_start_  = 0;
  ik_id_end_    = 0;

  IK_test  = new ManipulatorKinematicsDynamics(ARM);
}

RobotisState::~RobotisState()
{
}

bool RobotisState::setInverseKinematics(int cnt, int all_steps, Eigen::MatrixXd start_rotation, double start_phi)
{
  for (int dim = 0; dim < 3; dim++)
    ik_target_position_.coeffRef(dim, 0) = calc_task_tra_.coeff(cnt, dim);
  
  Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

  Eigen::Quaterniond target_quaternion(kinematics_pose_msg_.pose.orientation.w,
                                       kinematics_pose_msg_.pose.orientation.x,
                                       kinematics_pose_msg_.pose.orientation.y,
                                       kinematics_pose_msg_.pose.orientation.z);
  Eigen::Matrix3d goal_rotation = robotis_framework::convertQuaternionToRotation(target_quaternion);

  if(cnt == 0)
  {
    is_inv = false;
    int c;
    bool ik_s;
    Eigen::Vector3d test_position;
    Eigen::MatrixXd test_rotation;
    Eigen::VectorXd Old_JointAngle(8);
    float test_phi;
    float test_slide_pos;
    for(float i=0; i<=12; i++)
    {
      c = int(((double)all_steps-1) * i/12);
      for (int dim = 0; dim < 3; dim++)
        test_position.coeffRef(dim, 0) = calc_task_tra_.coeff(c, dim);
      Eigen::Quaterniond q = slerp(i/12, start_quaternion, target_quaternion, is_inv);
      test_rotation = robotis_framework::convertQuaternionToRotation(q);
      test_phi = start_phi + i/12 * (kinematics_pose_msg_.phi - start_phi);
      test_slide_pos = calc_slide_tra_.coeff(c,0);
      // for (int id = 0; id < idx.size(); id++)
      //   Old_JointAngle[idx[id]] = manipulator_link_data_[idx[id]]->joint_angle_;
      ik_s = IK_test->InverseKinematics_p2p(test_position, test_rotation, test_phi, test_slide_pos, Old_JointAngle, bool(i));
      if(!ik_s){
        if(!is_inv){
          is_inv = true;
          i = -1;
        }else
          return false;
      }
    }
  }


  double count = (double) cnt / (double) all_steps;

  ik_target_quaternion = slerp(count, start_quaternion, target_quaternion, is_inv);

  ik_target_phi_ = start_phi + count * (kinematics_pose_msg_.phi - start_phi);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(ik_target_quaternion);
  return true;
}

Eigen::Quaterniond RobotisState::slerp(double t, Eigen::Quaterniond& self, Eigen::Quaterniond& other, bool inv)
{
  using std::acos;
  using std::sin;
  using std::abs;
  static const double one = 1 - 0.0001;

  double d = self.dot(other);
  double absD = abs(d);
  
  double scale0;
  double scale1;

  if(absD>=one)
  {
    scale0 = 1 - t;
    scale1 = t;
  }
  else 
  {
    // theta is the angle between the 2 quaternions
    double theta = acos(absD);
    
    double sinTheta = sin(theta);
    scale0 = sin( ( 1 - t ) * theta) / sinTheta;
    scale1 = sin( ( t * theta) ) / sinTheta;
  }
  if(inv)
    if(d>=0) scale1 = -scale1;
  else
    if(d<0) scale1 = -scale1;
  Eigen::Quaterniond q;
  q.coeffs() = (scale0 * self.coeffs()) + (scale1 * other.coeffs());
  q.coeffs() /= q.norm();
  // Eigen::Quaterniond qq = q;
  
  // qq.coeffs() *= -1;
  // d = 180*acos(other.dot(q))/M_PI;
  // double dd = 180*acos(other.dot(qq))/M_PI; 
  // double a = (q.coeffs()-other.coeffs()).norm();
  // double b = (qq.coeffs()-other.coeffs()).norm();
  // // d = fabs(q.coeffs().outer(other.coeffs()));
  // // double dd = fabs(qq.coeffs().outer(other.coeffs()));
  // std::cout<<d<<" "<<dd<<" "<<a<<" "<<b<<" "<<a+b-2<<std::endl;
  return q;
}

