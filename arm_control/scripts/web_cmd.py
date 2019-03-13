#!/usr/bin/env python

import rospy
from arm_control import ArmTask
from std_msgs.msg import String, Float64
from arm_control.msg import WebCmd


def set_sub():
    print "[Arm] web control: " 
    web_cmd_sub = rospy.Subscriber(
        '/web_cmd',
        WebCmd,
        web_cmd_callback,
        queue_size=10
    )


def web_cmd_callback(msg):
    try:
        rospy.loginfo('send cmd b')
        if 'absolute' in msg.cmd:
            arm[msg.name].ikMove(msg.mode, msg.pose, msg.euler, msg.value)
        elif 'relative' in msg.cmd:
            arm[msg.name].relative_move_pose(msg.mode, msg.pose)
        elif 'noa_move' in msg.cmd:
            arm[msg.name].noa_move_suction(msg.mode, n=msg.noa[0], o=msg.noa[1], a=msg.noa[2])
        elif 'set_speed' in msg.cmd:
            arm[msg.name].set_speed(msg.value)
        elif 'single_joint' in msg.cmd:
            arm[msg.name].singleJointMove(msg.joint, msg.value)
        elif 'joint_move' in msg.cmd:
            arm[msg.name].jointMove(msg.value, msg.pose)
        elif 'back_home' in msg.cmd:
            arm[msg.name].back_home()
        rospy.loginfo('send cmd')
    except KeyError:
        rospy.logerr('arm -> KeyError')


if __name__ == '__main__':
    rospy.init_node('web_cmd')

    arm_right = ArmTask('right_arm')
    arm_left  = ArmTask('left_arm')
    arm = {"right": arm_right, "left": arm_left}
    set_sub()

    rospy.spin()
