#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import roslib
roslib.load_manifest('mobile_platform')
import rospy

''' stage 1 '''
''' imu '''
# from lib.strategy import Strategy

''' rfid '''
# from lib.strategy_rfid import Strategy

''' qrcode '''
# from lib.strategy_qr import Strategy

''' stage 2 '''
''' imu '''
# from lib.strategy2 import Strategy
''' rfid '''
# from lib.strategy2_rfid import Strategy

''' qrcode '''
from lib.strategy2_qr import Strategy

''' stage 3 '''
''' imu '''
''' rfid '''

'''
    input: scan_black/qrcode_angle (type:std_msgs/Float32)
           scan_black/scaninfo (type:mobile_platform.msg/scaninfo)
                dis (error distance)
                angle (error angle)
                scanstate (模擬紅外線感測器)
'''

def main():
    rospy.init_node('scan_strategy', anonymous=True)
    robot = Strategy()

    # 30 hz
    rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        if(robot._param.start):
            robot.Process()
        else:
            # print("Don't start up Strategy")
            robot.Robot_Stop()
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
