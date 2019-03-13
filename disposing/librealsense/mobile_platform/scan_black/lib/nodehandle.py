#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import rospy
import rospkg
import subprocess

# rostopic 
from mobile_platform.msg import scaninfo
from std_msgs.msg import Int32,Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# FILENAME 

FILENAME = rospkg.RosPack().get_path('mobile_platform')+'/config/'+'test1.yaml'

class NodeHandle(object):
    '''
        image process
            img: 影像
            start: 影像執行
            imgShow: 顯示影像
            loadParam: check new parameter
        decide grid parameter
            middleY: 垂直高度 (0~影像rows)
            range: 一半寬度
            threshold: 二值化權重
            weight: 數位感測權重
            scanNum: 數位感測數量
        direction error parameter
            sliceNum: 圖像分割數 
    '''
    def __init__(self):
        self.__img = None
        self.__start = 1
        self.__imgShow = 1

        self.__middleY = 240
        self.__range = 30
        self.__threshold = 100
        self.__weight = 50
        self.__scanNum = 7

        self.__sliceNum = 10

        self.__loadParam = False

        self.Load_Param()

        # publish
        self.pub_scaninfo = rospy.Publisher('scan_black/scaninfo',scaninfo, queue_size = 1)

        # subscriber
        rospy.Subscriber("usb_cam/image_raw",Image,self.Sub_Img)

        rospy.Subscriber("scan_black/scanSave",Bool,self.Save_Param)
        rospy.Subscriber("scan_black/scanStart",Bool,self.Sub_Start)
        rospy.Subscriber("scan_black/scanImgShow",Bool,self.Sub_Show)

        rospy.Subscriber("scan_black/middleY",Int32,self.Sub_MiddleY)
        rospy.Subscriber("scan_black/range",Int32,self.Sub_Range)
        rospy.Subscriber("scan_black/threshold",Int32,self.Sub_Threshold)
        rospy.Subscriber("scan_black/weight",Int32,self.Sub_Weight)
        rospy.Subscriber("scan_black/scanNum",Int32,self.Sub_ScanNum)
        rospy.Subscriber("scan_black/sliceNum",Int32,self.Sub_SilceNum)

    def Sub_Img(self,msg):
        self.__img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    def Sub_Start(self,msg):
        self.__start = msg.data
    def Sub_Show(self,msg):
        self.__imgShow = msg.data

    def Sub_MiddleY(self,msg):
        self.__middleY = msg.data
        self.__loadParam = True
    def Sub_Range(self,msg):
        self.__range = msg.data
        self.__loadParam = True
    def Sub_Threshold(self,msg):
        self.__threshold = msg.data
        self.__loadParam = True
    def Sub_Weight(self,msg):
        self.__weight = msg.data
        self.__loadParam = True
    def Sub_ScanNum(self,msg):
        self.__scanNum = msg.data
        self.__loadParam = True
    def Sub_SilceNum(self,msg):
        self.__sliceNum = msg.data
        self.__loadParam = True
    
    def Save_Param(self,msg):
        self.Set_Param()
        if (rospy.has_param('mobile_platform')):
            print('dump')
            subprocess.call(['rosparam','dump',FILENAME,'/mobile_platform'])
            # self.Load_Param()
        else:
            print('Not found')
            

    def Load_Param(self):
        if (rospy.has_param('mobile_platform/scan_black/middleY')):
            self.__middleY = rospy.get_param("mobile_platform/scan_black/middleY")
        if (rospy.has_param('mobile_platform/scan_black/range')):
            self.__range = rospy.get_param("mobile_platform/scan_black/range")
        if (rospy.has_param('mobile_platform/scan_black/threshold')):
            self.__threshold = rospy.get_param("mobile_platform/scan_black/threshold")
        if (rospy.has_param('mobile_platform/scan_black/weight')):
            self.__weight = rospy.get_param("mobile_platform/scan_black/weight")
        if (rospy.has_param('mobile_platform/scan_black/scanNum')):
            self.__scanNum = rospy.get_param("mobile_platform/scan_black/scanNum")
        if (rospy.has_param('mobile_platform/scan_black/sliceNum')):
            self.__sliceNum = rospy.get_param("mobile_platform/scan_black/sliceNum")

    def Set_Param(self):
        rospy.set_param('mobile_platform/scan_black/middleY', self.__middleY)
        rospy.set_param('mobile_platform/scan_black/range', self.__range)
        rospy.set_param('mobile_platform/scan_black/threshold', self.__threshold)
        rospy.set_param('mobile_platform/scan_black/weight', self.__weight)
        rospy.set_param('mobile_platform/scan_black/scanNum', self.__scanNum)
        rospy.set_param('mobile_platform/scan_black/sliceNum', self.__sliceNum)

    @property
    def img(self):
        return self.__img
    @img.setter
    def img(self, value):
        self.__img = value

    @property
    def start(self):
        return self.__start
    @start.setter
    def start(self, value):
        self.__start = value

    @property
    def imgShow(self):
        return self.__imgShow
    @imgShow.setter
    def imgShow(self, value):
        self.__imgShow = value

    '''
        parameter  
    '''
    @property
    def middleY(self):
        return self.__middleY

    @middleY.setter
    def middleY(self, value):
        self.__middleY = value
    
    @property
    def range(self):
        return self.__range

    @range.setter
    def range(self, value):
        self.__range = value
    
    @property
    def threshold(self):
        return self.__threshold

    @threshold.setter
    def threshold(self, value):
        self.__threshold = value
    
    @property
    def weight(self):
        return self.__weight

    @weight.setter
    def weight(self, value):
        self.__weight = value
    
    @property
    def scanNum(self):
        return self.__scanNum

    @scanNum.setter
    def scanNum(self, value):
        self.__scanNum = value

    @property
    def sliceNum(self):
        return self.__sliceNum

    @sliceNum.setter
    def sliceNum(self, value):
        self.__sliceNum = value

    '''  
        check load new parameter
    '''     
    @property
    def loadParam(self):
        return self.__loadParam

    @loadParam.setter
    def loadParam(self, value):
        self.__loadParam = value
    