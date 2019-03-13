#include "manipulator_h_base_module/slide_control.h"
#include <string>

using namespace robotis_manipulator_h;

slide_control::slide_control()
{
    ros::NodeHandle nh_private("~");
    gazebo_mode = nh_private.param<bool>("gazebo", false);
    if(gazebo_mode)
    {
        std::string side_str  = nh_private.param<std::string>("side", "");
        std::string side_label = "right" == side_str ? "r": "l";
        slide_pos_pub = n.advertise<std_msgs::Float64>("/mobile_dual_arm/" + side_label + "_slide_position/command", 5);
    }
    else
    {
        slide_cmd_pub = n.advertise<manipulator_h_base_module_msgs::SlideCommand>("/slide_command_msg", 1);
    }
    slide_pos = 0;
    goal_slide_pos = 0;
    result_slide_pos = 0;
}
slide_control::~slide_control(){}

void slide_control::slide_pub()
{
    if(gazebo_mode)
    {
        std_msgs::Float64 msg;
        msg.data = result_slide_pos;
        slide_pos_pub.publish(msg);
        slide_pos = result_slide_pos;
    }
    else
    {
        manipulator_h_base_module_msgs::SlideCommand slide_msg;
        slide_msg.pos = result_slide_pos;
        slide_cmd_pub.publish(slide_msg);
    }
}

void slide_control::slideFeedback(const linear_motion::Slide_Feedback::ConstPtr& msg)
{
    is_busy = msg->is_busy;
    slide_pos = -(double)msg->curr_pos / 100000.0;
}