#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import roslib
roslib.load_manifest('mobile_platform')
import rospy

# insert opencv 3.0 
# import sys
# sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import math
import numpy as np
import cv2

# rostopic msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from scan_black.msg import scaninfo

# define behavior 
MOBILE_ROBOT = 0
PLATFORM = 1

class NodeHandle(object):
    def __init__(self):
        self._dis = None
        self._scanstate = None
        self._start = 1
        self.behavior = MOBILE_ROBOT
        
        self.pub_cmdvel = rospy.Publisher('motion/cmd_vel',Twist, queue_size = 1)

        self.sub_scaninfo = rospy.Subscriber("scan_black/scaninfo",scaninfo,self.Set_ScanInfo)
        self.sub_start = rospy.Subscriber("scan_black/strategy_start",Int32,self.Set_Start)

    def Set_ScanInfo(self,msg):
        self._dis = msg.dis
        self._scanstate = msg.scanstate
    
    def Set_Start(self,msg):
        self._start = msg.data
    
    def Set_Behavior(self,msg):
        self.behavior = msg.data

class Strategy(NodeHandle):
    def __init__(self):
        super(Strategy,self).__init__()
        self._maxV = 10
        self._minV = 12
        self._kp = 6
        self._ki = 0.8
        self._kd = 4.0
        self.prevIntegral = 0
        self.lastError = 0
    def Process(self):
        if(self.behavior == MOBILE_ROBOT):
            if(self._scanstate):
                scanNum = len(self._scanstate)
                count = self._scanstate.count(1)
                if(count <= math.ceil((scanNum)*(1./3))):
                    
                    # method 1
                    # if(abs(self._dis) <= 270):
                    #     x = self._minV*math.cos(self.Deg2Rad((self._dis/3.0)))
                    #     y = self._minV*math.sin(self.Deg2Rad((self._dis/3.0)))
                    #     yaw = 0
                    # else:
                    #     if(self._dis > 0):
                    #         x = self._minV*math.cos(self.Deg2Rad((270/3.0)))
                    #         y = self._minV*math.sin(self.Deg2Rad((270/3.0)))
                    #     else:
                    #         x = self._minV*math.cos(self.Deg2Rad(-(270/3.0)))
                    #         y = self._minV*math.sin(self.Deg2Rad(-(270/3.0)))                            
                    #     yaw = 0
                    # print(x,y,yaw)

                    # method 2
                    # self._dis = self._dis/5
                    # self.prevIntegral += self._dis
                    # derivative = self._dis - self.lastError
                    # self.lastError = self._dis
                    # turn = self._kp*self._dis+self._ki*self.prevIntegral+self._kd*derivative
                    # turn = turn/1000.0
                    
                    # x = (self._minV+turn)*math.cos(self.Deg2Rad((self._dis/3.0)))
                    # y = (self._minV-turn)*math.sin(self.Deg2Rad((self._dis/3.0)))
                    # yaw = 0
                    # print(x,y,yaw,turn)

                    # method 3
                    # 可視範圍 約 53 ~ -53 
                    angle = self._dis / 53.0

                    # self._dis = self._dis/5
                    self.prevIntegral += self._dis
                    derivative = self._dis - self.lastError
                    self.lastError = self._dis
                    turn = self._kp*self._dis+self._ki*self.prevIntegral+self._kd*derivative
                    turn = turn/1000.0

                    x = (self._minV - abs(turn))*math.cos(math.radians(angle)) #- turn*math.sin(math.radians(angle))
                    y = self._minV*math.sin(math.radians(angle)) + turn*math.cos(math.radians(angle))
                    yaw = angle/53.0
                    # yaw = 0
                    #self.Robot_Vel([x,y,yaw])
                    #print(x,y,yaw)
                    self.Robot_Vel([y,-x,yaw])
                    print(y,-x,yaw)
                else:
                    print('stop')
                    self.behavior = PLATFORM
            else:
                print('No Scan Info !!!!!!')
        elif(self.behavior == PLATFORM):
            self.Robot_Stop()
    def Deg2Rad(self,deg):
        return deg*math.pi/180

    def Robot_Stop(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        self.pub_cmdvel.publish(vel)

    def Robot_Vel(self,vec):
        vel = Twist()
        vel.linear.x = vec[0]
        vel.linear.y = vec[1]
        vel.angular.z = vec[2]
        self.pub_cmdvel.publish(vel)

def main():
    rospy.init_node('scan_strategy', anonymous=True)
    strategy = Strategy()

    # 30 hz
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if(strategy._start):
            strategy.Process()
        else:
            pass
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
