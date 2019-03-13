#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import rospy
import rospkg
import subprocess

# rostopic 
from std_msgs.msg import Bool,Int32
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

FILENAME = rospkg.RosPack().get_path('mobile_platform')+'/config/'+'vision.yaml'

class NodeHandle(object):
    '''
        camera
            img
        setting
            start
            imgShow
        qr 
            ang
        param
            threshold
            cannyMax
            cannyMin
    '''
    def __init__(self):
        self.__img = None

        self.__start = 1
        self.__imgShow = 1

        self.__ang = 999

        self.__threshold = 120
        self.__cannyMin = 30
        self.__cannyMax = 100

        self.__loadParam = False

        self.Load_Param()

        # publish
        self.pub_qrangle = rospy.Publisher('scan_black/qrcode_angle',Pose2D, queue_size = 1)

        # subscriber
        rospy.Subscriber("usb_cam/image_raw",Image,self.Sub_Img)

        rospy.Subscriber("scan_black/qrcode_start",Bool,self.Sub_Start)
        rospy.Subscriber("scan_black/qrcode_save",Bool,self.Save_Param)
        rospy.Subscriber("scan_black/qrImgShow",Bool,self.Sub_Show)

        rospy.Subscriber("scan_black/qrcode_threshold",Int32,self.Sub_Threshold)
        rospy.Subscriber("scan_black/qrcode_cannyMin",Int32,self.Sub_CannyMin)
        rospy.Subscriber("scan_black/qrcode_cannyMax",Int32,self.Sub_CannyMax)

    def Sub_Img(self,msg):
        self.__img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    def Sub_Start(self,msg):
        self.__start = msg.data
    def Sub_Show(self,msg):
        self.__imgShow = msg.data

    def Sub_Threshold(self,msg):
        self.__threshold = msg.data
    def Sub_CannyMin(self,msg):
        self.__cannyMin = msg.data
    def Sub_CannyMax(self,msg):
        self.__cannyMax = msg.data


    def Save_Param(self,msg):
        self.Set_Param()
        if (rospy.has_param('mobile_platform/vision/')):
            print('dump')
            subprocess.call(['rosparam','dump',FILENAME,'/mobile_platform/vision/'])
            # self.Load_Param()
        else:
            print('Not found')

    def Load_Param(self):
        if (rospy.has_param('mobile_platform/vision/qrcode/threshold')):
            self.__threshold = rospy.get_param("mobile_platform/vision/qrcode/threshold")
        if (rospy.has_param('mobile_platform/vision/qrcode/cannyMin')):
            self.__cannyMin = rospy.get_param("mobile_platform/vision/qrcode/cannyMin")
        if (rospy.has_param('mobile_platform/vision/qrcode/cannyMax')):
            self.__cannyMax = rospy.get_param("mobile_platform/vision/qrcode/cannyMax")
    
    def Set_Param(self):
        rospy.set_param('mobile_platform/vision/qrcode/threshold', self.__threshold)
        
        if(self.__cannyMin <= self.__cannyMax):
            rospy.set_param('mobile_platform/vision/qrcode/cannyMin', self.__cannyMin)
            rospy.set_param('mobile_platform/vision/qrcode/cannyMax', self.__cannyMax)
        else:
            rospy.set_param('mobile_platform/vision/qrcode/cannyMin', self.__cannyMax)
            rospy.set_param('mobile_platform/vision/qrcode/cannyMax', self.__cannyMin)

    """ camera """
    @property
    def img(self):
        return self.__img
    @img.setter
    def img(self, value):
        self.__img = value
    
    """ setting """
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
    
    """ qr """
    @property
    def ang(self):
        return self.__ang

    @ang.setter
    def ang(self, value):
        self.__ang = value
    
    """ qr param """
    @property
    def threshold(self):
        return self.__threshold

    @threshold.setter
    def threshold(self, value):
        self.__threshold = value

    @property
    def cannyMin(self):
        return self.__cannyMin

    @cannyMin.setter
    def cannyMin(self, value):
        self.__cannyMin = value
    @property
    def cannyMax(self):
        return self.__cannyMax

    @cannyMax.setter
    def cannyMax(self, value):
        self.__cannyMax = value