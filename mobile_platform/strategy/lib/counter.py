#!/usr/bin/env python3
# -*- coding: utf-8 -*-+


import rospy

class TimeCounter(object):
    '''
        COUNTER
                output: time,state
        CALCULATOR
                output: time,_
    '''
    def __init__(self,behavior='COUNTER',time = 1.0):
        self.__begin = 0
        self.__behavior = behavior
        self.__time = time
    
    def Init(self,reset = False):
        if(reset):
            self.__begin = 0
        else:
            self.__begin = rospy.get_rostime()
    
    def Set_Time(self,time):
        self.__time = time
    
    def Process(self):
        t_ = rospy.get_rostime()

        if(self.__begin == 0):
            self.Init()

        if(self.__behavior == 'COUNTER'):
            time, state = self.Counter(t_)
        elif(self.__behavior == 'CALCULATOR'):
            pass
        return time, state

    def Counter(self,t):
        time = t.to_sec() - self.__begin.to_sec()

        if(time < self.__time):
            return time, False
        else:
            self.Init(True)
            return time, True
