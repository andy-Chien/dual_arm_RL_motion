#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import math
import numpy as np


class PIDControl(object):
    def __init__(self):
        self._kp = 20
        self._ki = 0.1
        self._kd = 5.0
        self.prevIntegral = 0
        self.lastError = 0
    
    def Process(self,dis,ang,maxVel,minVel,yaw):
        # ang = (dis/10.0) / 53.0
        self.prevIntegral += dis
        derivative = dis - self.lastError
        self.lastError = dis
        turn = self._kp*dis+self._ki*self.prevIntegral+self._kd*derivative
        turn = turn/10000.0

        x = (minVel - abs(turn))*math.cos(math.radians(ang)) - turn*math.sin(math.radians(ang))
        y = minVel*math.sin(math.radians(ang)) + turn*math.cos(math.radians(ang))

        # yaw = yaw*((ang/20.0)/math.sqrt(1+pow(ang/20.0,2.0)))
        yaw = turn
        return x,y,yaw
        

class PIDControl_Y(object):
    def __init__(self):
        # self._kp = 61.0
        # self._ki = 0.91
        # self._kd = 30
        self._kp = 30.0
        self._ki = 0.33
        self._kd = 22.0
        self.prevIntegral = 0
        self.lastError = 0
    
    def Init(self):
        self.prevIntegral = 0
        self.lastError = 0
    
    def Process(self,dis,ang,minVel):
        # ang = (dis/10.0) / 53.0
        dis /= 10.0

        self.prevIntegral += dis
        derivative = dis - self.lastError
        self.lastError = dis
        turn = self._kp*dis+self._ki*self.prevIntegral+self._kd*derivative
        y = turn/1000.0

        if(y > minVel-2):
            y = minVel-2
        elif(y < -(minVel-2)):
            y = -(minVel-2)
        
        # if(dis > 75):
        #     return y
        # elif(dis < -75):
        #     return y
        # else:
        #     return 0.0

        if(dis > 30):
            return y
        elif(dis < -30):
            return y
        else:
            return 0.0

        # return y

class PIDControl_Yaw(object):
    def __init__(self):
        self._kp = 10.0
        self._ki = 0.01
        self._kd = 2.0
        self.prevIntegral = 0
        self.lastError = 0
    
    def Init(self):
        self.prevIntegral = 0
        self.lastError = 0
    
    def Process(self,ang,yaw):
        # print(ang)
        self.prevIntegral += ang
        derivative = ang - self.lastError
        self.lastError = ang
        turn = self._kp*ang+self._ki*self.prevIntegral+self._kd*derivative
        turn = turn/30.0

        # if(turn >= (yaw+2)):
        #     turn = (yaw+2)
        # elif(turn <= -(yaw+2)):
        #     turn = -(yaw+2)

        return turn

class PIDControl_Qr(object):
    def __init__(self,kp,ki,kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.prevIntegral = 0
        self.lastError = 0
    
    def Init(self):
        self.prevIntegral = 0
        self.lastError = 0
    
    def Process(self,dis,ang,minVel):

        self.prevIntegral += dis
        derivative = dis - self.lastError
        self.lastError = dis
        turn = self._kp*dis+self._ki*self.prevIntegral+self._kd*derivative
        y = turn/500.0

        # if(y > minVel-2):
        #     y = minVel-2
        # elif(y < -(minVel-2)):
        #     y = -(minVel-2)

        if(dis > 20):
            return y
        elif(dis < -20):
            return y
        else:
            return 0.0

        # return y
    