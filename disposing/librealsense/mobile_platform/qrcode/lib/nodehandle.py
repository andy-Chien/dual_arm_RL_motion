#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import rospy
import rospkg

# rostopic 
from std_msgs.msg import Float32,Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class NodeHandle(object):
    '''
        
    '''
    def __init__(self):
        self.__img = None

        self.__start = 0
        self.__ang = 999
        self.__dis = None

        self.__loadParam = False

        # publish
        self.pub_qrangle = rospy.Publisher('scan_black/qrcode_angle',Float32, queue_size = 1)

        # subscriber
        rospy.Subscriber("usb_cam/image_raw",Image,self.Sub_Img)

        rospy.Subscriber("scan_black/qrcode_start",Bool,self.Sub_Start)

    def Sub_Img(self,msg):
        self.__img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    def Sub_Start(self,msg):
        self.__start = msg.data

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
    def ang(self):
        return self.__ang

    @ang.setter
    def ang(self, value):
        self.__ang = value
    
    @property
    def dis(self):
        return self.__dis

    @dis.setter
    def dis(self, value):
        self.__dis = value