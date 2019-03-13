#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import rospy
import rospkg
import subprocess

# rostopic msg
from mobile_platform.msg import scaninfo
from std_msgs.msg import Int32,Float32,Float64,Bool,String
from geometry_msgs.msg import Twist

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

# FILENAME 
FILENAME = rospkg.RosPack().get_path('mobile_platform')+'/config/'+'stage1_imu.yaml'
# FILENAME = rospkg.RosPack().get_path('mobile_platform')+'/config/'+'stage1_rfid.yaml'

class NodeHandle(object):
    '''
        strategy
            start: 策略執行
            behavior: 機器人行為狀態
            loadParam: check new parameter
        robot param
            maxVel: 機器人最高速度 (目前沒用到)
            minVel: 機器人最低速度
            velYaw: 機器人走直線旋轉校正速度
            rotateYaw: 機器人定點旋轉速度
            crossTime: 穿過停止點
        error param
            errorRotate0: 機器人旋轉0度誤差
            errorRotate90: 機器人旋轉90度誤差
            rotateSlowAng: 機器人旋轉減速角度
            errorAng: 校正誤差
            errorMoibledis: 機器人追線時 低於距離(pixel)校正車頭角
            errorMoibleAng: 機器人追線時 校正誤差
            errorCorrectionDis: 
        scan infomation
            dis: 各點平均誤差(pixel)
            ang: 線與車頭之夾角
            scanstate: 紅外線數位值
        IMU
            qrang: 180 ~ -180 degree
        RFID
            stopPoint
                "0": home
                "1": first point
                "2": second point
    '''
    def __init__(self):
        self.__start = 0
        self.__behavior = INIT
        self.__loadParam = False

        self.__maxVel = 10.0
        # self.__minVel = 15.0
        # self.__velYaw = 15.0
        self.__minVel = 25.0
        self.__velYaw = 25.0
        self.__rotateYaw = 45.0
        self.__crossTime = 0.5

        self.__errorRotate0 = 8.0
        self.__errorRotate90 = 83.0
        self.__rotateSlowAng = 20
        self.__errorAng = 3.0
        self.__errorMoibledis = 1000
        self.__errorMoibleAng = 3.0
        self.__errorCorrectionDis = 600
        
        self.__dis = None
        self.__ang = None
        self.__scanState = None

        self.__qrang = None

        self.__stopPoint = 999

        self.Load_Param()

        self.pub_cmdvel = rospy.Publisher('motion/cmd_vel',Twist, queue_size = 1)
        self.pub_behavior = rospy.Publisher('scan_black/strategy_behavior',Int32, queue_size = 1)
        self.pub_dualArm = rospy.Publisher('scan_black/dualarm_start',Bool, queue_size = 1)
        self.pub_voice = rospy.Publisher('scan_black/voice_start',Bool, queue_size = 1)
        self.pub_startCamera = rospy.Publisher('scan_black/scanStart',Bool, queue_size = 1)
        self.pub_resetImu = rospy.Publisher('scan_black/resetImu',Bool, queue_size = 1)

        rospy.Subscriber("scan_black/strategy_start",Bool,self.Sub_Start)
        rospy.Subscriber("scan_black/strategy_behavior",Int32,self.Sub_Behavior)
        rospy.Subscriber("scan_black/strategy_save",Bool,self.Save_Param)

        rospy.Subscriber("scan_black/scaninfo",scaninfo,self.Sub_ScanInfo)
        # rospy.Subscriber("scan_black/qrcode_angle",Float32,self.Sub_QRAngle)
        rospy.Subscriber("/imu_data",Float64,self.Sub_QRAngle)
        rospy.Subscriber("/rfid",String,self.Sub_RFID)
    
    def Sub_ScanInfo(self,msg):
        self.__dis = msg.dis
        self.__ang = msg.angle
        self.__scanState = msg.scanstate
    def Sub_Start(self,msg):
        self.__start = msg.data
    def Sub_Behavior(self,msg):
        self.__behavior = msg.data
        self.__loadParam = True
    def Sub_QRAngle(self,msg):
        self.__qrang = msg.data
    def Sub_RFID(self,msg):
        self.__stopPoint = msg.data

    def Save_Param(self,msg):
        # self.Set_Param()
        if (rospy.has_param('mobile_platform')):
            print('dump')
            subprocess.call(['rosparam','dump',FILENAME,'/mobile_platform'])
            self.Load_Param()
        else:
            print('Not found')
    
    def Load_Param(self):
        if (rospy.has_param('mobile_platform/strategy/maxVel')):
            self.__maxVel = rospy.get_param("mobile_platform/strategy/maxVel")
        if (rospy.has_param('mobile_platform/strategy/minVel')):
            self.__minVel = rospy.get_param("mobile_platform/strategy/minVel")
        if (rospy.has_param('mobile_platform/strategy/velYaw')):
            self.__velYaw = rospy.get_param("mobile_platform/strategy/velYaw")
        if (rospy.has_param('mobile_platform/strategy/rotateYaw')):
            self.__rotateYaw = rospy.get_param("mobile_platform/strategy/rotateYaw")
        if (rospy.has_param('mobile_platform/strategy/crossTime')):
            self.__crossTime = rospy.get_param("mobile_platform/strategy/crossTime")
        
        if (rospy.has_param('mobile_platform/strategy/errorRotate0')):
            self.__errorRotate0 = rospy.get_param("mobile_platform/strategy/errorRotate0")
        if (rospy.has_param('mobile_platform/strategy/errorRotate90')):
            self.__errorRotate90 = rospy.get_param("mobile_platform/strategy/errorRotate90")
        if (rospy.has_param('mobile_platform/strategy/rotateSlowAng')):
            self.__rotateSlowAng = rospy.get_param("mobile_platform/strategy/rotateSlowAng")
        if (rospy.has_param('mobile_platform/strategy/errorAng')):
            self.__errorAng = rospy.get_param("mobile_platform/strategy/errorAng")
        if (rospy.has_param('mobile_platform/strategy/errorMoibledis')):
            self.__errorMoibledis = rospy.get_param("mobile_platform/strategy/errorMoibledis")
        if (rospy.has_param('mobile_platform/strategy/errorMoibleAng')):
            self.__errorMoibleAng = rospy.get_param("mobile_platform/strategy/errorMoibleAng")
        if (rospy.has_param('mobile_platform/strategy/errorCorrectionDis')):
            self.__errorCorrectionDis = rospy.get_param("mobile_platform/strategy/errorCorrectionDis")
    
    def Set_Param(self):
        rospy.set_param('mobile_platform/strategy/maxVel', self.__maxVel)
        rospy.set_param('mobile_platform/strategy/minVel', self.__minVel)
        rospy.set_param('mobile_platform/strategy/velYaw', self.__velYaw)
        rospy.set_param('mobile_platform/strategy/rotateYaw', self.__rotateYaw)
        rospy.set_param('mobile_platform/strategy/crossTime', self.__crossTime)

        rospy.set_param('mobile_platform/strategy/errorRotate0', self.__errorRotate0)
        rospy.set_param('mobile_platform/strategy/errorRotate90', self.__errorRotate90)
        rospy.set_param('mobile_platform/strategy/rotateSlowAng', self.__rotateSlowAng)
        rospy.set_param('mobile_platform/strategy/errorAng', self.__errorAng)
        rospy.set_param('mobile_platform/strategy/errorMoibledis', self.__errorMoibledis)
        rospy.set_param('mobile_platform/strategy/errorMoibleAng', self.__errorMoibleAng)
        rospy.set_param('mobile_platform/strategy/errorCorrectionDis', self.__errorCorrectionDis)
    
    ''' strategy '''
    @property
    def start(self):
        return self.__start

    @start.setter
    def start(self, value):
        self.__start = value

    @property
    def behavior(self):
        return self.__behavior

    @behavior.setter
    def behavior(self, value):
        self.__behavior = value
    
    @property
    def loadParam(self):
        return self.__loadParam

    @loadParam.setter
    def loadParam(self, value):
        self.__loadParam = value

    ''' robot param '''
    @property
    def maxVel(self):
        return self.__maxVel

    @maxVel.setter
    def maxVel(self, value):
        self.__maxVel = value
    
    @property
    def minVel(self):
        return self.__minVel

    @minVel.setter
    def minVel(self, value):
        self.__minVel = value

    @property
    def velYaw(self):
        return self.__velYaw

    @velYaw.setter
    def velYaw(self, value):
        self.__velYaw = value
    
    @property
    def rotateYaw(self):
        return self.__rotateYaw

    @rotateYaw.setter
    def rotateYaw(self, value):
        self.__rotateYaw = value
    
    @property
    def crossTime(self):
        return self.__crossTime

    @crossTime.setter
    def crossTime(self, value):
        self.__crossTime = value


    ''' error param '''
    @property
    def errorRotate0(self):
        return self.__errorRotate0

    @errorRotate0.setter
    def errorRotate0(self, value):
        self.__errorRotate0 = value
    
    @property
    def errorRotate90(self):
        return self.__errorRotate90

    @errorRotate90.setter
    def errorRotate90(self, value):
        self.__errorRotate90 = value
    
    @property
    def rotateSlowAng(self):
        return self.__rotateSlowAng

    @rotateSlowAng.setter
    def rotateSlowAng(self, value):
        self.__rotateSlowAng = value
    
    @property
    def errorAng(self):
        return self.__errorAng

    @errorAng.setter
    def errorAng(self, value):
        self.__errorAng = value
    
    @property
    def errorMoibledis(self):
        return self.__errorMoibledis

    @errorMoibledis.setter
    def errorMoibledis(self, value):
        self.__errorMoibledis = value

    @property
    def errorMoibleAng(self):
        return self.__errorMoibleAng

    @errorMoibleAng.setter
    def errorMoibleAng(self, value):
        self.__errorMoibleAng = value
    
    @property
    def errorCorrectionDis(self):
        return self.__errorCorrectionDis

    @errorCorrectionDis.setter
    def errorCorrectionDis(self, value):
        self.__errorCorrectionDis = value
    

    ''' scan infomation '''
    @property
    def dis(self):
        return self.__dis

    @dis.setter
    def dis(self, value):
        self.__dis = value
    
    @property
    def ang(self):
        return self.__ang

    @ang.setter
    def ang(self, value):
        self.__ang = value
    
    @property
    def scanState(self):
        return self.__scanState

    @scanState.setter
    def scanState(self, value):
        self.__scanState = value
    
    
    ''' IMU '''
    @property
    def qrang(self):
        return self.__qrang

    @qrang.setter
    def qrang(self, value):
        self.__qrang = value

    ''' RFID '''
    @property
    def stopPoint(self):
        return self.__stopPoint

    @stopPoint.setter
    def stopPoint(self, value):
        self.__stopPoint = value
    
    
    

        