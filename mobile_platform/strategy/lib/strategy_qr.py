#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import math
import numpy as np


# lib
from lib.nodehandle import NodeHandle
from lib.pidcontrol import PIDControl,PIDControl_Y,PIDControl_Yaw,PIDControl_Qr
from lib.fuzzycontrol import FUZZYControl
from lib.counter import TimeCounter

# rostopic msg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Int32


# define behavior 
MOBILE_ROBOT = 0
CORRECTION = 1
PLATFORM = 2
NEXT_POINT = 3
HOME = 4
MANUAL = 5
ROTATE = 6
GO_POINT = 7
RETURN_POINT = 8
CROSS = 9
INIT = 10
DELIVERY = 11
ORDER = 12
ARM_MOVE = 13

# FLAG 
CONTROL = 'PIDCONTROL'
# CONTROL = 'FUZZYCONTROL'

'''
    HOME -> FIRST
        INIT -> MOBILE -> CORRECTION_0 -> ROTATE_90 -> CORRECTION_90 -> PLATFORM
    FIRST -> SECOND 
        NEXT -> ROTATE_0 -> CROSS -> MOBILE -> CORRECTION_0 -> ROTATE_90 -> CORRECTION_90 -> PLATFORM
    POINT -> HOME
        HOME -> ROTATE -> CROSS_FIRST -> MOBILE -> PLATFORM
'''

class Strategy(object):
    '''
        Offset track(目前停止)
            prev_dis:
            prev_ang:
            prev_vel:
        CONTROL
            initPID: 初始化不要使PID不斷累加
        QRCODE(目前沒用到)
            state:
            pre_state:
            not_find: 
        ROTATE
            rotateAng: 目標角度
        CROSS
            timer: 計數器
        HOME
            homeFlag
                1: go home
                0: 前進
            homeTimes: 記錄走到的停止點
    '''
    def __init__(self):
        self._param = NodeHandle()
        if(CONTROL == 'PIDCONTROL'):
            self.control = PIDControl()
            self.controlY = PIDControl_Y()
            self.controlYaw = PIDControl_Yaw()

            # self.controlQRX = PIDControl_Qr(20.0,0.05,10.0)
            # self.controlQRY = PIDControl_Qr(20.0,0.05,10.0)
            # self.controlQRX = PIDControl_Qr(15,0.05,12.0)
            self.controlQRX = PIDControl_Qr(15,0.05,14.0)
            self.controlQRY = PIDControl_Qr(30.0,0.05,15.0)
        elif(CONTROL == 'FUZZYCONTROL'):
            self.control = FUZZYControl()
        
        self.prev_dis = 0
        self.prev_ang = 0
        self.prev_vel = []

        self.initPID = 0

        self.state = 0
        self.pre_state = 0
        self.not_find = 0

        self.stopTimes = 0

        self.dualArm = 0
        
        ''' rotate  '''
        self.rotateAng = self._param.errorRotate0
        self.pre_rotateYaw = 0

        ''' cross '''
        self.timer = TimeCounter(time = self._param.crossTime)
        self.timerQr = TimeCounter(time = 1.0)
        self.timerQrFlag = False
        ''' home '''
        self.homeFlag = 0
        self.homeTimes = 0

    def Process(self):
        if(self._param.behavior == MOBILE_ROBOT):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Mobile_Strategy()

        elif(self._param.behavior == CORRECTION):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Correction_Strategy()

        elif(self._param.behavior == PLATFORM):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Platform_Strategy()

        elif(self._param.behavior == NEXT_POINT):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Next_Point_Strategy()

        elif(self._param.behavior == HOME):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Home_Strategy()
            print('HOME')

        elif(self._param.behavior == MANUAL):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.state = 0
            self.initPID = 0
            self.controlYaw.Init()
            self.controlY.Init()
            print('MANUAL')

        elif(self._param.behavior == ROTATE):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Rotate_Strategy()

        elif(self._param.behavior == GO_POINT):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Go_Point_Strategy()
            print('GO_POINT')

        elif(self._param.behavior == RETURN_POINT):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Return_Point_Strategy()
            print('RETURN_POINT')
        elif(self._param.behavior == CROSS):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Cross_Strategy()
        elif(self._param.behavior == INIT):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.Init_Strategy()
            print('Init')
        elif(self._param.behavior == ARM_MOVE):
            if(self._param.loadParam):
                self.Change_Behavior()
            self.ARM_MOVE_Strategy()
            print('ARM_MOVE')
        else:
            print("Don't have Behavior")
            self.Robot_Stop()

    def Mobile_Strategy(self):
        if(self._param.scanState):
            count = self._param.scanState.count(1)
            if(count):
                scanNum = len(self._param.scanState)
                if(count <= math.ceil((scanNum)*(2./3)) and self._param.stopPoint == 999):
                    self.state = 0
                    # Method 3
                    #if(CONTROL == 'PIDCONTROL'):
                    #    x,y,yaw = self.control.Process(self._param.dis,self._param.ang,self._param.maxVel,self._param.minVel,self._param.velYaw)
                    #elif(CONTROL == 'FUZZYCONTROL'):
                    #    x,y,yaw = self.control.Process(self._param.dis,self._param.ang)
                    # yaw = 0
                    #self.Robot_Vel([y,-x,yaw])
                    #print(y,-x,yaw)

                    # Method 4
                    # x,y,yaw = self.control.Process(self._param.dis,self._param.ang,self._param.maxVel,self._param.minVel,self._param.velYaw)
                    # if(abs(self._param.ang) > 10.0):
                    #     if(self._param.ang > 0):
                    #         x = -(self._param.minVel*math.cos(math.radians(self._param.ang)))*0.15
                    #         y = -(self._param.minVel*math.sin(math.radians(self._param.ang)))*0.15
                    #         # yaw = self._param.velYaw
                    #         yaw = (self._param.velYaw+abs(yaw))
                    #     else:
                    #         x = -(self._param.minVel*math.cos(math.radians(self._param.ang)))*0.15
                    #         y = (self._param.minVel*math.sin(math.radians(self._param.ang)))*0.15
                    #         # yaw = -self._param.velYaw
                    #         yaw = -(self._param.velYaw+abs(yaw))
                    # else:
                    #     x,y,_ = self.control.Process(self._param.dis,self._param.ang,self._param.maxVel,self._param.minVel,self._param.velYaw)
                    #     # x,y,_ = self.control.Process(self._param.dis,self._param.ang)
                    #     yaw = 0
                        
                    ''' Method 5 '''
                    y = self.controlY.Process(self._param.dis,self._param.ang,self._param.minVel)
                    x = (self._param.minVel - abs(y))*math.cos(math.radians(self._param.ang)) - y*math.sin(math.radians(self._param.ang))
                    
                    if(abs(self._param.dis) > self._param.errorMoibledis):
                        yaw = 0
                    else:       
                        if(abs(self._param.ang) > self._param.errorMoibleAng):
                            yaw = self.controlYaw.Process(self._param.ang,self._param.velYaw)
                        else:
                            yaw = 0
                    if(self.homeFlag == 0):
                        self.Robot_Vel([x,y,yaw])
                        print(x,y,yaw)
                    else:
                        self.Robot_Vel([-x,y,yaw])
                        print(-x,y,yaw)
                    # print(self._param.ang)
                    # self.prev_dis = self._param.dis
                    # self.prev_ang = self._param.ang
                    # self.prev_vel = [x,y,yaw]
                elif(self._param.stopPoint != 999 and self._param.stopPoint != '91' and self._param.stopPoint != '90'):
                    print('STOP')
                    self.state = 1
                    self.Robot_Stop()

                    if(self.homeFlag == 1):
                        self._param.behavior = HOME
                    elif(self.homeTimes == int(self._param.stopPoint)):
                        self._param.behavior = CROSS
                    elif(self._param.stopPoint == '2'):
                        self._param.behavior = CORRECTION
                        print('state 2')
                        self.dualArm = 2
                        self.timerQrFlag = False
                        # self.Dual_Arm_Start_2()
                    else:
                        self.dualArm = 1
                        self._param.behavior = CORRECTION
                        self.homeTimes += 1
                        self.timerQrFlag = False
                if(self.stopTimes >= 3):
                    self._param.stopPoint = 999
                    self.stopTimes = 0
                else:
                    self.stopTimes += 1
                self.pre_state = self.state
            else:
                print('Offset track !!!!!!')
                if(len(self.prev_vel)):
                    if(self.prev_vel[2] == 0):
                        x = -(self.prev_vel[0])*0.8
                        y = -self.prev_vel[1]*1.5
                        yaw = 0
                    else:     
                        x = (self._param.minVel*math.cos(math.radians(self.prev_ang)))*0.5
                        y = self._param.minVel*math.sin(math.radians(self.prev_ang))
                        yaw = 0
                else:
                    x = 0
                    y = 0
                    yaw = 0
                    print('No scan line')
                # self.Robot_Vel([y,-x,yaw])
                self.Robot_Stop()
        else:
            print('No Scan Info !!!!!!')
            self.Robot_Stop()

    def Correction_Strategy(self):
        # y = self.controlY.Process(self._param.dis,self._param.ang,self._param.minVel)
        if(self._param.qrX == None):
            print('fuck!!!!!!!!!!!!!!!!')
            dis = 0
        else:
            # print('fuck!!!!!!!!!!!!!!!!',self._param.errorCorrectionDis)
            # dis = math.sqrt(math.pow(self._param.qrX,2.0)+math.pow(self._param.qrY,2.0))
            dis = self._param.qrY
        if(self.timerQrFlag == False):
            time,self.timerQrFlag = self.timerQr.Process()
        else:    
            # if(self._param.dis < self._param.errorCorrectionDis):
            if(abs(dis) < self._param.errorCorrectionDis):
                if(self._param.qrTheta is not None and self._param.qrTheta != 999):
                    RPang = self.Norm_Angle(self.rotateAng - self._param.qrTheta)
                    if(abs(RPang) > self._param.errorAng):
                        if(RPang > 0):
                            x = 0
                            y = 0
                            yaw = self._param.velYaw
                            # yaw = self._param.rotateYaw
                        else:
                            x = 0
                            y = 0
                            yaw = -self._param.velYaw
                            # yaw = -self._param.rotateYaw

                        self.Robot_Vel([x,y,yaw])
                        print('CORRECTION','FRONT',self._param.qrTheta)
                    else:
                        self.Robot_Stop()
                        self.Robot_Stop()
                        self.Robot_Stop()

                        print('CORRECTION',self.rotateAng,self._param.errorRotate0)
                        if(self.dualArm == 1):
                        # if(self.dualArm == 1 or self.dualArm == 2):
                            if(self.rotateAng == self._param.errorRotate0):
                                self._param.behavior = ROTATE
                                self.rotateAng = self._param.errorRotate90
                            else:
                                self._param.behavior = PLATFORM
                                self.rotateAng = self._param.errorRotate0
                                self.initPID = 1
                        elif(self.dualArm == 2):
                            if(self.rotateAng == self._param.errorRotate0):
                                self._param.behavior = PLATFORM
            
                        
                    self.not_find = 0
                else:
                    print('CORRECTION not find')
                    if(self.not_find < 100):
                        self.not_find += 1
                        self.Robot_Stop()
                    else:
                        self.not_find = 0
                        if(self.dualArm == 1):
                            if(self.rotateAng == self._param.errorRotate0):
                                self._param.behavior = ROTATE
                                self.rotateAng = self._param.errorRotate90
                            else:
                                self._param.behavior = PLATFORM
                                self.rotateAng = self._param.errorRotate0
                                self.initPID = 1
                        elif(self.dualArm == 2):
                            if(self.rotateAng == self._param.errorRotate0):
                                self._param.behavior = PLATFORM  
                            self.initPID = 1
                self._param.qrTheta = 999
            else:
                # x = 0
                yaw = 0
                # y = 0
                x = self.controlQRX.Process(self._param.qrY,self._param.qrTheta,self._param.minVel)
                y = self.controlQRY.Process(self._param.qrX,self._param.qrTheta,self._param.minVel)
                self.Robot_Vel([x,y,yaw])
                print('CORRECTION','dis',x,y)
                self._param.qrX = None
    
    def Platform_Strategy(self):
        print('PLATFORM')
        self.state = 0
        if(self.initPID):
            self.controlYaw.Init()
            self.controlY.Init()
            self.initPID = 0
        self.Robot_Stop()
        if(self.homeFlag == 0 and self.dualArm == 1):
            self.Dual_Arm_Start_1()
        elif(self.homeFlag == 0 and self.dualArm == 2):
            self.Dual_Arm_Start_2()
    
    def Next_Point_Strategy(self):
        print('NEXT_POINT')
        self.Robot_Stop()
        self._param.behavior = ROTATE
        self.rotateAng = self._param.errorRotate0
        self.dualArm = 0
        
            
    def Rotate_Strategy(self):
        print('ROTATE fuck!!!!!!',self._param.errorRotate90)
        # yaw = self.controlYaw(self._param.qrTheta,self._param.velYaw)
        if(self._param.qrTheta is not None and self._param.qrTheta != 999):
            RPang = self.Norm_Angle(self.rotateAng - self._param.qrTheta)
            x = self.controlQRX.Process(self._param.qrY,self._param.qrTheta,self._param.minVel)
            y = self.controlQRY.Process(self._param.qrX,self._param.qrTheta,self._param.minVel)
            if(abs(RPang) > self._param.errorAng and RPang > self._param.rotateSlowAng):
                if(RPang > 0):
                    # x = 0
                    # y = 0
                    yaw = self._param.velYaw
                    self.pre_rotateYaw = yaw
                    # yaw = self._param.velYaw*0.8
                    # yaw = 0
                    # yaw = self._param.rotateYaw
                else:
                    # x = 0
                    # y = 0
                    yaw = self._param.velYaw
                    self.pre_rotateYaw = yaw
                    # yaw = -self._param.velYaw*0.8
                    # yaw = 0
                    # yaw = -self._param.rotateYaw
                self.Robot_Vel([x,y,yaw])
                # print('ROTATE','angle',self._param.qrTheta)
                print('ROTATE','vector',x,y)
            elif((abs(RPang) > self._param.errorAng and RPang <= self._param.rotateSlowAng)):
                if(RPang > 0):
                    # x = 0
                    # y = 0
                    yaw = self._param.velYaw
                    self.pre_rotateYaw = yaw
                    # yaw = self._param.velYaw*0.8
                    # yaw = 0
                    # yaw = self._param.rotateYaw*0.8
                else:
                    # x = 0
                    # y = 0
                    yaw = -self._param.velYaw
                    self.pre_rotateYaw = yaw
                    # yaw = self._param.velYaw*0.8
                    # yaw = 0
                    # yaw = -self._param.rotateYaw*0.8
                self.Robot_Vel([x,y,yaw])
                # print('ROTATE','angle',self._param.qrTheta)
                print('ROTATE','vector',x,y)
            else:
                self.Robot_Stop()
                self.Robot_Stop()
                self.Robot_Stop()
                if(self.rotateAng == self._param.errorRotate90):
                    self._param.behavior = CORRECTION
                    print('ROTATE COREECTION')
                else:
                    self._param.behavior = CROSS
                    print('ROTATE CROSS')
            self.not_find = 0
        else:
            print('ROTATE not find')
            if(self.not_find < 100):
                self.not_find += 1
                x = 0
                y = 0
                self.Robot_Vel([x,y,self.pre_rotateYaw])
                # self.Robot_Stop()
            else:
                self.not_find = 0
                if(self.rotateAng == self._param.errorRotate90):
                    self._param.behavior = CORRECTION
                    print('ROTATE COREECTION')
                else:
                    self._param.behavior = CROSS
                    print('ROTATE CROSS')
        self._param.qrTheta = 999
    
    def Go_Point_Strategy(self):
        time,state = self.timer.Process()

        if(state):
            self.Robot_Stop()
            self._param.behavior = CORRECTION
        else:
            x = self._param.minVel
            y = 0
            yaw = 0
            self.Robot_Vel([x,y,yaw])

    def Return_Point_Strategy(self):
        time,state = self.timer.Process()

        if(state):
            self.Robot_Stop()
            self._param.behavior = ROTATE
            self.rotateAng = self._param.errorRotate0
        else:
            x = -self._param.minVel
            y = 0
            yaw = 0
            self.Robot_Vel([x,y,yaw])
    
    def Cross_Strategy(self):
        print('CROSS')
        time,state = self.timer.Process()

        if(state):
            self.Robot_Stop()
            self._param.behavior = MOBILE_ROBOT
            self.rotateAng = self._param.errorRotate0
        elif(state == 0 and self.homeFlag == 0):
            x = self._param.minVel
            y = 0
            yaw = 0
            self.Robot_Vel([x,y,yaw])
        elif(state == 0 and self.homeFlag == 1):
            x = -self._param.minVel
            y = 0
            yaw = 0
            self.Robot_Vel([x,y,yaw])
        # if(self.pre_state == 1 and self.state == 0):
        #     if(self._param.scanState):
        #         if(self._param.qrTheta is not None and self._param.qrTheta != 999):
        #             x = self._param.minVel
        #             y = 0
        #             yaw = 0
        #             self.not_find = 0
        #             # self.Robot_Vel([y,-x,yaw])
        #             self.Robot_Vel([x,y,yaw])
        #         elif(self.not_find > 60):
        #             self.Robot_Stop()
        #             self._param.behavior = MOBILE_ROBOT
        #             self.not_find = 0
        #                 # print('next point not find line')
        #         else:
        #             self.not_find +=1
        #             x = self._param.minVel
        #             y = 0
        #             yaw = 0
        #             # self.Robot_Vel([y,-x,yaw])
        #             self.Robot_Vel([x,y,yaw])
        #         self._param.qrTheta = 999
        # else:
        #     self.Robot_Stop()
        #     print('fuck Cross')
    
    def Init_Strategy(self):
        self.rotateAng = self._param.errorRotate0
        self.homeFlag = 0
        self.homeTimes = 0
        self.Robot_Stop()
        self._param.behavior = MOBILE_ROBOT
        self._param.stopPoint = 999
        # self.Reset_IMU()

    def Home_Strategy(self):
        print('HOME times',self.homeTimes,'HOME stop',self._param.stopPoint)
        if(self.homeFlag == 0):
            print('HOME',1)
            self.homeFlag = 1
            self.Robot_Stop()
            self._param.behavior = ROTATE
            self.rotateAng = self._param.errorRotate0
            self.homeTimes -= 1
        else:
            if(self.homeTimes == 0 and self._param.stopPoint == '0'):
                print('home')
                self.Robot_Stop()
                self._param.behavior = PLATFORM
            else:
                if(self.homeTimes == int(self._param.stopPoint)):
                    self.homeTimes -= 1
                    self._param.behavior = CROSS
                elif(self._param.stopPoint == 999):
                    self._param.behavior = CROSS
                else:
                    self._param.behavior = MOBILE_ROBOT
    
    def ARM_MOVE_Strategy(self):
        if(self._param.armMove == 1):
            self.Robot_Vel([12,0,0])
        elif(self._param.armMove == -1):
            self.Robot_Vel([-12,0,0])
        else:
            print('arm move fuck !!!!')

            
    def Deg2Rad(self,deg):
        return deg*math.pi/180
    
    def Norm_Angle(self,angle):
        if(angle > 180):
            angle -= 360
        elif(angle < -180):
            angle +=360
        return angle

    def Robot_Stop(self):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        self._param.pub_cmdvel.publish(vel)

    def Robot_Vel(self,vec):
        vel = Twist()
        vel.linear.x = vec[0]
        vel.linear.y = vec[1]
        vel.angular.z = vec[2]
        self._param.pub_cmdvel.publish(vel)
    
    def Change_Behavior(self):
        self.Robot_Stop()
        self.Robot_Stop()
        self._param.loadParam = False
    
    def Dual_Arm_Start_1(self):
        start = Int32()
        start.data = 1
        self._param.pub_dualArm1.publish(start)
    
    def Dual_Arm_Start_2(self):
        start = Int32()
        start.data = 2
        self._param.pub_dualArm1.publish(start)

    def Scan_Camera_Start(self):
        start = Bool()
        start.data = True
        self._param.pub_startCamera.publish(start)

    def Scan_Camera_Stop(self):
        start = Bool()
        start.data = False
        self._param.pub_startCamera.publish(start)
    
    def Reset_IMU(self):
        reset = Bool()
        reset.data = True
        self._param.pub_resetImu.publish(reset)
    

