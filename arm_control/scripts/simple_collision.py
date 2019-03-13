#!/usr/bin/env python

import rospy
from arm_control import ArmTask

areaAbove = False
areaBelow = False
whoInsideAbove = 'right or left'
whoInsideBelow = 'right or left'

rospy.init_node('simple_collision')
print("runing simple_collision")

right_arm = ArmTask('right_arm')
left_arm  = ArmTask('left_arm')
rospy.on_shutdown(lambda: (right_arm.freeze(False), left_arm.freeze(False)))
rospy.sleep(.5)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    right_arm_pos = right_arm.get_fb().group_pose.position
    left_arm_pos  = left_arm.get_fb().group_pose.position
    if right_arm_pos.x > 0 and right_arm_pos.y > -0.15 and -0.25 > right_arm_pos.z > -0.7 or right_arm.wait:
        if not areaAbove:
            if right_arm.wait:
                right_arm.freeze(False)
                right_arm.wait = False
                rospy.loginfo('right arm resume')
            areaAbove = True
            whoInsideAbove = 'right'
        elif whoInsideAbove != 'right':
            right_arm.freeze(True)
            right_arm.wait = True
            rospy.logwarn('right arm waitting')
    elif whoInsideAbove == 'right':
        whoInsideAbove = ''
        areaAbove = False

    if left_arm_pos.x > 0 and left_arm_pos.y < 0.15 and -0.25 > left_arm_pos.z > -0.7 or left_arm.wait:
        if not areaAbove:
            if left_arm.wait:
                left_arm.freeze(False)
                left_arm.wait = False
                rospy.loginfo('left arm resume')
            areaAbove = True
            whoInsideAbove = 'left'
        elif whoInsideAbove != 'left':
            left_arm.freeze(True)
            left_arm.wait = True
            rospy.logwarn('left arm waitting')
    elif whoInsideAbove == 'left':
        whoInsideAbove = ''
        areaAbove = False

    # if right_arm_pos.y > -0.05 and right_arm_pos.z > -0.2:
    #     if not areaBelow:
    #         areaBelow = True
    #         whoInsideBelow = 'right'
    #     elif whoInsideBelow != 'right':
    #         right_arm.__wait_pub.publish(True)
    #         right_arm.__is_wait = True

    # if left_arm_pos.y < 0.05 and left_arm_pos.z > -0.2:
    #     if not areaBelow:
    #         areaBelow = True
    #         whoInsideBelow = 'left'
    #     elif whoInsideBelow != 'left':
    #         left_arm.__wait_pub.publish(True)
    #         left_arm.__is_wait = True
    
    # if right_arm.wait and not areaAbove:
    #     right_arm.freeze(False)
    #     right_arm.wait = False
    #     rospy.loginfo('right arm resume')

    # if left_arm.wait and not areaAbove:
    #     left_arm.freeze(False)
    #     left_arm.wait = False
    #     rospy.loginfo('left arm resume')

    rate.sleep()
