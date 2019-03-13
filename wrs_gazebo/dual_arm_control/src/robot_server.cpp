#include "ros/ros.h"
#include "dual_arm_control/robot.h"

bool add(dual_arm_control::robot::Request  &req,
         dual_arm_control::robot::Response &res)
{
  //res.sum = req.a + req.b;
  ROS_INFO("request: slide_R=%f,
					 joint1_R=%f,
					 joint2_R=%f,
					 joint3_R=%f,
					 joint4_R=%f,
					 joint5_R=%f,
					 joint6_R=%f,
					 joint7_R=%f,
					 gripper_R=%f,
					 slide_L=%f,
					 joint1_L=%f,
					 joint2_L=%f,
					 joint3_L=%f,
					 joint4_L=%f,
					 joint5_L=%f,
					 joint6_L=%f,
					 joint7_L=%f,
					 gripper_L=%f",(long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("HI", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
