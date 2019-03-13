#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dual_arm_control/robot_msg.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher robot_pub = n.advertise<dual_arm_control::robot_msg>("chatter", 1000);

	ros::Rate loop_rate(10);	
	while (ros::ok())
	{
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
		//std_msgs::String msg;
		dual_arm_control::robot_msg msg;
    	//std::stringstream ss;
    	//ss << "hello world " << count;
    	msg.A = 10.21;

    	//ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    	robot_pub.publish(msg);

    	ros::spinOnce();

    	loop_rate.sleep();
	}
	return 0;
}

