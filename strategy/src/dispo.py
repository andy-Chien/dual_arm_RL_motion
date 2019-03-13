#!/usr/bin/env python
"""Use to generate arm task and run."""

TIMELIMIT = 6 *60

global TIMEOUT  
TIMEOUT = False

SPEED = 12

EXPIRED = 'ABCD'




Y_MAX = 1.04
Y_MIN = 9.05







































import os
import sys
import copy
import time
import threading
from math import radians, degrees, sin, cos, pi, acos, asin
import numpy as np
import rospy
from std_msgs.msg import Bool, Int32, String
from arm_control import ArmTask, SuctionTask
from disposing_vision.msg import coordinate_normal
from yolov3_sandwich.msg import ROI
from geometry_msgs.msg import Twist

QRCODEPOS = (0.06, 0.4, -0.45)

idle              = 0
busy              = 1
initPose          = 2
frontSafetyPos    = 3
rearSafetyPos     = 4
move2CamPos1      = 10
move2CamPos2      = 11
move2CamPos3      = 12
move2CamPos4      = 13
startCam          = 20
moveCam           = 22
watch             = 24
move2Object1      = 30
move2Object2      = 31
move2Object3      = 32
move2Object4      = 33
move2Object0      = 36
pickObject        = 34
grasping          = 35
leaveShelfTop1    = 40
leaveShelfTop2    = 41
leaveShelfTop3    = 42
leaveShelfTop4    = 43
move2QRcode       = 50
checkDate         = 51
move2Shelf1       = 60
move2Shelf2       = 62
move2Shelf3       = 63
move2Shelf4       = 64
move2PlacePos     = 65
PlaceObject       = 66
PlaceObject1      = 67
leaveShelfMiddle1 = 70
leaveShelfMiddle2 = 71
leaveShelfMiddle3 = 72
leaveShelfMiddle4 = 73
move2Bin1         = 80
leaveBin1         = 90

class disposingTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        self.en_sim = False
        self.__set_pubsub()
        if len(sys.argv) >= 2:
            print(type(sys.argv[1]))
            if sys.argv[1] == 'True':
                rospy.set_param('self.en_sim', sys.argv[1])
                self.en_sim = rospy.get_param('self.en_sim')
        self.name = _name
        self.state = initPose
        self.nowState = initPose 
        self.nextState = idle
        self.reGripCnt = 0
        self.pickList = 0
        self.pickListAll = 8
        self.arm = ArmTask(self.name + '_arm')
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        self.sucAngle = 0
        self.sandwitchPos = [0, 0, 0]
        self.sandwitchEu  = [0, 0, 0]
        self.sandwitchSuc = 0
        self.sandwitchVec = [0, 0, 0]
        self.camMovePos = [0, 0]
        self.Qrcode     = ''
        self.checkCnt = 0
        self.moveCnt  = 0
        self.mode = 'p2p'

        self.ROI_Pos = [0, 640, 0, 480]
        self.Vision_pos = [0, 0, 0, 0, 0, 0]
        if self.name == 'right':
            self.is_right = 1
        if self.name == 'left':
            self.is_right = -1
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)

    def __set_pubsub(self):
        self.__realsense_sub = rospy.Subscriber(
            '/object/normal',
            coordinate_normal,
            self.disposing_vision_callback,
            queue_size=1
        )
        self.__ROI_sub = rospy.Subscriber(
            '/object/ROI',
            ROI,
            self.ROI_callback,
            queue_size=1
        )
        self.__Qrcode_sub =  rospy.Subscriber(
            '/barcode',
            String,
            self.Qrcode_callback,
            queue_size=1
        ) 
        #pub 2
        self.mobileStade_pub = rospy.Publisher(
            '/scan_black/strategy_behavior',
            Int32,
            queue_size=1
        )
        self.moveWheel_pub = rospy.Publisher(
            '/motion/cmd_vel',
            Twist,
            queue_size=1
        )
        self.mobileMove_pub = rospy.Publisher(
            'scan_black/strategy_forward',
            Int32,
            queue_size=1
        )

    def ROI_callback(self, msg):
        self.ROI_Pos = [msg.x_min,msg.x_Max,msg.y_min,msg.y_Max]

    def Qrcode_callback(self, msg):
        self.Qrcode = msg.data

    def disposing_vision_callback(self, msg):
        if msg.normal_z < -0.5:
            self.Vision_pos = [msg.x,msg.y,msg.z,msg.normal_x,msg.normal_y,msg.normal_z]

    def ROI_regulate(self):
        Img_Obj_x = (self.ROI_Pos[0]+self.ROI_Pos[1])/2
        Img_Obj_y = (self.ROI_Pos[2]+self.ROI_Pos[3])/2
        # Img_Obj_Center = (Img_Obj_x,Img_Obj_y) 

        if Img_Obj_x < 180:
            self.camMovePos[0] = 1
        elif Img_Obj_x > 320:
            self.camMovePos[0] = -1
        else:
            self.camMovePos[0] = 0

        if Img_Obj_y < 180:
            self.camMovePos[1] = -1
        elif Img_Obj_y > 300:
            self.camMovePos[1] = 1
        else:
            self.camMovePos[1] = 0

    def VisiontoArm(self):
        Img_Pos = np.mat([[self.Vision_pos[0]],[self.Vision_pos[1]],[self.Vision_pos[2]],[1]])
        Img_nVec = np.mat([[self.Vision_pos[3]],[self.Vision_pos[4]],[self.Vision_pos[5]],[0]])

        # Img_Pos = np.mat([[0], [0.01], [0.3]])
        # Img_nVec = np.mat([[0.86], [0], [-0.5]])

        dx = 0      # unit:meter
        dy = 0.055 # unit:meter
        dz = -0.12  # unit:meter 
        TransMat_EndToImg = np.mat([[1,0,0,dx],[0,0,1,dy],[0,-1,0,dz],[0,0,0,1]])
        
        ori = left.arm.get_fb().orientation
        T0_7 = np.identity(4)
        for i in range(0,4):
            for j in range(0,4):
                T0_7[i][j] = ori[i*4+j]

        Mat_nVec_Pos = np.mat([ [Img_nVec[0,0], Img_Pos[0,0]],
                                [Img_nVec[1,0], Img_Pos[1,0]],
                                [Img_nVec[2,0], Img_Pos[2,0]],
                                [      0      ,        1   ]])
    
        Mat_VecPos_ImgToBase = T0_7 * TransMat_EndToImg * Mat_nVec_Pos
        self.sandwitchPos = Mat_VecPos_ImgToBase[0:3, 1]
        self.sandwitchVec = Mat_VecPos_ImgToBase[0:3, 0]
        print 'T0_7', T0_7
        print 'self.sandwitchPos', self.sandwitchPos
        print 'self.sandwitchVec', self.sandwitchVec

    @property
    def finish(self):
        return self.pickList == self.pickListAll

    def getPlacePos(self):
        self.pos = [0, 0.7, -0.8]
        self.euler = [-90, 0, 60]
        self.phi = -30
        self.sucAngle = 0

    def move_to_vector_point(self, pos, vector=[1,0,0]): # This funthion will move arm and return suction angle 
    # Only for left arm Euler (0 0 30)
        goal_vec = [0, 0, 0]
        goal_vec[0], goal_vec[1], goal_vec[2] = -round(vector[0], 4), -round(vector[1], 4), -round(vector[2], 4)
        
        a = sin(pi/3)
        b = sin(pi/6)
        x, y, z = goal_vec[0], goal_vec[1], goal_vec[2]
        roll_angle = 0.0
        print 'goal_vec', goal_vec
        suc_angle = -round(acos(round((b*y - a*z) / (a*a + b*b), 4)), 4)
        print 'suc_angle', suc_angle
        roll_angle_c = round(acos(round(x / sin(suc_angle), 4)), 4)
        print 'asin', round(-(a*y + b*z)/((a*a + b*b) * sin(suc_angle)), 4)
        roll_angle_s = round(asin(round(-(a*y + b*z) / ((a*a + b*b) * sin(suc_angle)), 4)), 4)
        print 'roll_angle_c', roll_angle_c
        print 'roll_angle_s', roll_angle_s
        if roll_angle_s >= 0:
            roll_angle = roll_angle_c
        else:
            roll_angle = -roll_angle_c

        pos[0] += vector[0]*0.065
        pos[1] += vector[1]*0.065
        pos[2] += vector[2]*0.065
        euler = self.sandwitchEu
        euler[0] = degrees(roll_angle)
        euler[1] = 0
        euler[2] = 30
        self.sandwitchPos = pos
        self.sandwitchEu  = euler
        self.sandwitchSuc = degrees(suc_angle)

    @property
    def finish(self):
        return self.pickList == self.pickListAll

    def process(self): 
        global TIMEOUT 
        if self.arm.is_stop:                                       # must be include in your strategy 
            self.finish = True                                     # must be include in your strategy 
            print "!!! Robot is stop !!!"                          # must be include in your strategy 
            self.suction.gripper_vaccum_off()                      # must be include in your strategy 
            return                                                 # must be include in your strategy 
 
        if self.state == idle:
            if TIMEOUT:
                self.finish = True 
            if self.finish: 
                return 
            else: 
                self.state = move2CamPos1 
         
        elif self.state == busy: 
            if self.arm.is_busy: 
                return 
            else: 
                self.state = self.nextState 
                self.nowState = self.nextState 
                return 
 
        elif self.state == initPose:
            self.state = busy
            self.nextState = move2CamPos1 
            self.suction.gripper_suction_deg(0)
            self.arm.set_speed(SPEED)
            if TIMEOUT:
                self.nextState = idle
                self.finish = True

        elif self.state == move2CamPos1:
            self.state = busy
            self.nextState = move2CamPos2
            pos = (0, 0.45, -0.4)
            euler = (0, 0, -35)
            phi = 0
            if TIMEOUT:
                self.mode = 'line'
            self.arm.ikMove(self.mode, pos, euler, phi)
            self.suction.gripper_suction_deg(0)
            if TIMEOUT:
                self.nextState = idle
                self.finish = True

        elif self.state == move2CamPos2:
            self.state = busy
            self.nextState = move2CamPos3
            pos = (0, 0.86, -0.2)
            euler = (0, 0, 90)
            phi = 0
            self.arm.ikMove('line', pos, euler, phi)
            self.suction.gripper_suction_deg(-90)
            if TIMEOUT:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = move2Object0

        elif self.state == move2CamPos3:
            self.state = busy
            self.nextState = moveCam
            pos = (0, 0.965, -0.2)
            euler = (0, 0, 90)
            phi = 0
            self.arm.ikMove('p2p', pos, euler, phi)
            self.suction.gripper_suction_deg(-90)
            if TIMEOUT:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = move2Object0

        elif self.state == moveCam:
            if self.checkCnt == 0:
                self.ROI_Pos[0] = 99
                self.checkCnt = 1

            self.mobileStade_pub.publish(13)
            self.ROI_regulate()
            wheelCmd = Twist()
            self.arm.set_speed(10)
            if self.ROI_Pos[0] != 99:
                if self.camMovePos[0] == 0 and self.camMovePos[1] == 0:
                    # wheelCmd.linear.x = 0
                    # wheelCmd.linear.y = 0
                    self.arm.clear_cmd()
                    # self.moveWheel_pub.publish(wheelCmd)
                    self.mobileStade_pub.publish(2)
                    rospy.sleep(.1)
                    # print 'state1',  wheelCmd
                    self.state = watch
                    # self.moveWheel_pub.publish(wheelCmd)
                    self.mobileStade_pub.publish(2)
                    self.checkCnt = 0
                else:
                    # wheelCmd.linear.x = 12 * self.camMovePos[0]
                    # wheelCmd.linear.y = 0
                    msg = Int32()
                    msg.data = self.camMovePos[0]
                    self.mobileMove_pub.publish(msg)
                    # self.moveWheel_pub.publish(wheelCmd)
                    # print 'state2', wheelCmd
                    if not self.arm.is_busy and self.camMovePos[1] != 0:
                        # pos_y = 0.1 * self.camMovePos[1]
                        if self.camMovePos[1] > 0:
                            pos = (0, 1.035, -0.2)
                            euler = (0, 0, 90)
                            phi = 0
                            self.arm.ikMove('line', pos, euler, phi)
                        else:
                            pos = (0, 0.965, -0.2)
                            euler = (0, 0, 90)
                            phi = 0
                            self.arm.ikMove('line', pos, euler, phi)
                        # self.arm.relative_move_pose('line', [0 ,pos_y , 0])
                    curr_y = self.arm.get_fb().group_pose.position.y
                    if  self.arm.is_busy and self.camMovePos[1] == 0 or curr_y > Y_MAX or curr_y < Y_MIN:
                        self.arm.clear_cmd()
                        rospy.sleep(.1)
            else:
                self.checkCnt += 1
                if self.checkCnt > 30:
                    self.checkCnt = 1
                    wheelCmd.linear.x = 12
                    wheelCmd.linear.y = 0
                    rate = rospy.Rate(30)
                    for i in range(60):
                        msg = Int32()
                        msg.data = self.camMovePos[0]
                        self.mobileMove_pub.publish(msg)
                        # self.moveWheel_pub.publish(wheelCmd)
                        # print 'state3', wheelCmd
                        rate.sleep()
                    self.moveCnt += 1
                    # wheelCmd.linear.x = 0
                    # wheelCmd.linear.y = 0
                    self.mobileStade_pub.publish(2)
                    # self.moveWheel_pub.publish(wheelCmd)
                    # print 'state4', wheelCmd
                    print "self.moveCnt", self.moveCnt
                    if self.moveCnt >= 1:
                        TIMEOUT = True

            if TIMEOUT:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = move2Object0

        elif self.state == watch:
            self.state = move2Object0
            rospy.sleep(1)
            self.VisiontoArm()
            self.move_to_vector_point(self.sandwitchPos, self.sandwitchVec)
            if TIMEOUT:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = move2Object0
        
        elif self.state == move2Object0:
            self.state = busy
            self.nextState = move2Object1
            # self.arm.singleJointMove(index=0, pos=0)
            self.arm.set_speed(SPEED)
            self.arm.jointMove(0, (0, radians(-110), 0, radians(84), 0, radians(-50), 0))
            if TIMEOUT:
                self.nextState = move2CamPos1
            
        elif self.state == move2Object1:
            self.state = busy
            self.nextState = move2Object2
            pos = self.sandwitchPos[:]
            pos[2] = -0.3
            phi = -10
            self.arm.set_speed(SPEED)
            self.arm.ikMove('p2p', pos, self.sandwitchEu, phi)
            self.suction.gripper_suction_deg(self.sandwitchSuc)

        elif self.state == move2Object2:
            self.state = busy
            self.nextState = pickObject
            self.phi = -30
            self.arm.set_speed(SPEED)
            self.arm.noa_relative_pos('line', self.sandwitchPos, self.sandwitchEu, self.phi, self.sandwitchSuc, n=0, o=0, a=-0.02)

        elif self.state == pickObject:
            self.state = grasping
            self.suction.gripper_vaccum_on()
            self.arm.set_speed(20)
            self.arm.noa_move_suction('line', self.sandwitchSuc, n=0, o=0, a=0.15)
            rospy.sleep(.1)

        elif self.state == grasping:
            if self.suction.is_grip:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = busy
                self.nextState = leaveShelfTop1
                self.reGripCnt = 0
            elif not self.arm.is_busy:
                if self.en_sim:
                    self.state = busy
                    self.nextState = leaveShelfTop1
                else:
                    self.suction.gripper_vaccum_off
                    self.state = move2CamPos3

        elif self.state == leaveShelfTop1:
            self.state = busy
            self.nextState = leaveShelfTop2
            self.arm.set_speed(SPEED)
            self.arm.relative_move_pose('line', [0, 0, 0.1])

        elif self.state == leaveShelfTop2:
            self.state = busy
            self.nextState = move2QRcode
            pos = (0.06, 0.55, -0.3)
            euler = (0, 0, 0)
            phi = 0
            self.arm.ikMove('line', pos, euler, phi)
            self.suction.gripper_suction_deg(0)

        elif self.state == move2QRcode:
            self.state = busy
            self.nextState = checkDate
            pos = QRCODEPOS
            euler = (0, 0, -45)
            phi = 0
            self.arm.ikMove('p2p', pos, euler, phi)

        elif self.state == checkDate:
            if self.checkCnt == 0:
                self.Qrcode = ''
            if self.arm.is_busy is not True:
                euler = ((-40 + self.checkCnt*15), 0, 0)
                self.checkCnt += 1
                self.arm.set_speed(20)
                self.arm.move_euler('p2p', euler)
            if self.Qrcode != '' and self.Qrcode in EXPIRED or self.checkCnt >= 8 or TIMEOUT:
                self.arm.clear_cmd()
                rospy.sleep(.1)
                self.state = rearSafetyPos
                self.checkCnt = 0
            elif self.Qrcode != '':
                self.arm.clear_cmd()
                self.checkCnt = 0
                rospy.sleep(.1)
                self.state = move2Shelf1
            if self.suction.is_grip is not True:
                self.state == idle
                self.pickList += 1

        elif self.state == frontSafetyPos:
            self.state = busy
            self.nextState = move2Shelf1
            pos   = [0, 0.5, -0.5]
            euler = [-90, 0, 30]
            phi   = 0
            self.arm.ikMove('line', pos, euler, phi)

        elif self.state == rearSafetyPos:
            self.state = busy
            self.nextState = move2Bin1
            pos   = [0, 0.4, -0.5]
            euler = [90, -20, -30]
            phi   = 60
            self.arm.ikMove('line', pos, euler, phi)

        elif self.state == move2Shelf1:
            self.state = busy
            self.nextState = move2Shelf2 
            self.getPlacePos()
            self.pos[0] -= 0.05
            self.arm.set_speed(SPEED)
            self.arm.noa_relative_pos('p2p', self.pos, self.euler, self.phi, self.sucAngle, n=0, o=0, a=-0.05)

        elif self.state == move2Shelf2:
            self.state = busy
            self.nextState = PlaceObject
            self.getPlacePos()
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == leaveShelfMiddle1:
            self.state = busy
            self.nextState = idle 
            self.getPlacePos()
            self.pos[0] -= 0.05
            self.arm.set_speed(SPEED)
            self.arm.noa_relative_pos('line', self.pos, self.euler, self.phi, self.sucAngle, n=0, o=0, a=-0.1)
            self.pickList += 1
            if TIMEOUT:
                self.nextState = idle
                self.finish = True

        elif self.state == move2Bin1:
            self.state = busy
            self.nextState = PlaceObject1
            pos = (-0.3, 0.2, -0.45)
            euler = (90, -30, -30)
            phi = 60
            self.arm.ikMove('line', pos, euler, phi)

        elif self.state == leaveBin1:
            self.state = busy
            self.nextState = idle
            pos   = [0, 0.5, -0.5]
            euler = [90, -20, -30]
            phi   = 60
            self.arm.ikMove('line', pos, euler, phi)

        elif self.state == PlaceObject:
            self.state = leaveShelfMiddle1
            self.suction.gripper_vaccum_off()

        elif self.state == PlaceObject1:
            self.state = idle
            self.pickList += 1
            self.suction.gripper_vaccum_off()
            if TIMEOUT:
                self.nextState = idle
                self.finish = True

        # elif self.state == :

def start_callback(msg):
    global is_start
    if msg.data == 2 and not is_start:
        is_start = True


def count_time():
    task_start = time.time()
    task_count = time.time()-task_start
    while not rospy.is_shutdown() and task_count < TIMELIMIT:
        if task_count %1 == 0:
            print(task_count)
        task_count = time.time()-task_start
    TIMEOUT = True


if __name__ == '__main__':
    rospy.init_node('disopssing')        # enable this node

    counter = threading.Thread(target = count_time)
    counter.start()

    is_start = False
    rospy.Subscriber(
        'scan_black/dualarm_start_1',
        Int32,
        start_callback,
        queue_size=1
    )
    pub = rospy.Publisher(
        'scan_black/strategy_behavior',
        Int32,
        queue_size=1
    )

    left  = disposingTask('left')       # Set up left arm controller
    rospy.sleep(.3)

    while not rospy.is_shutdown() and not is_start:
        rospy.loginfo('waiting for start signal')
        rospy.sleep(.5)
    
    SuctionTask.switch_mode(True)

    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown() and not left.finish:
        left.process()
        rate.sleep()

    # robot arm back home
    if left.arm.is_stop is not True:
        rospy.loginfo('back home')
        left.arm.wait_busy()
        left.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))

        left.arm.wait_busy()
        left.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))

    SuctionTask.switch_mode(False)
    # publish finish signal to wheels
    pub.publish(4)
        
