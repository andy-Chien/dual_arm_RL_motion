#!/usr/bin/env python

"""Use to generate arm task and run."""

import os
import sys
import copy
from math import degrees

import rospy
from std_msgs.msg import Bool, Int32
from arm_control import ArmTask, SuctionTask





PICKORDER = 0
SPEED_R     = 60
SPEED_L     = 70
LUNCHBOX_H = 0.045
# The lesser one
lunchQuan = 1              
drinkQuan = 1
riceQuan  = 2

idle            = 0
busy            = 1
initPose        = 2
frontSafetyPos  = 3
rearSafetyPos   = 4
move2Bin        = 5
move2Shelf      = 6
moveIn2Shelf    = 7
leaveBin        = 8
leaveShelf      = 9
move2Object     = 10
move2PlacedPos  = 11
pickObject      = 12
placeObject     = 13
safePose1       = 14
safePose2       = 15
safePose3       = 16 
riceballEuler   = 17
rearSafetyPos2  = 18
leavePlacePos   = 19
grasping        = 20
missObj         = 21
safePose4       = 22

objectName = ['lunchbox', 'lunchbox', 'lunchbox', 'lunchbox',
              'drink',    'drink',    'drink',    'drink',
              'riceball', 'riceball', 'riceball', 'riceball']

lunchboxPos = [[-0.43,  0.165, -0.69],
               [-0.42,  0.15, -0.69],
               [-0.42,  0.15, -0.69],
               [-0.42,  0.15, -0.69]]

drinkPos =    [[-0.2, 0.09, -0.61],
               [-0.295, 0.09, -0.61],                   
               [-0.2, 0.19, -0.61],                              
               [-0.295, 0.19, -0.61]]

riceballPos = [[-0.17,  -0.221, -0.69],
               [-0.267,  -0.223, -0.69],
               [-0.17,  -0.1, -0.69],                             
               [-0.27,  -0.1, -0.69]]

lunchboxEu = [150, 0, 0]

drinkEu =    [0, 0, 0]
            
riceballXXEu = [45, 0, 0]
riceballEu   = [30, 0, 0]

               
objectPos = [lunchboxPos, drinkPos, riceballPos]
objectEu  = [lunchboxEu,  drinkEu,  riceballEu]

topRight    = [0.365, -0.1, -0.225]
topLeft     = [0.365,  0.1, -0.225]
middleRight = [0.445, -0.1, -0.545]
middleLeft  = [0.445,  0.1, -0.545]
bottomRight = [0.5, -0.2, -1.015]
bottomLeft  = [0.5,  0.2, -1.015]

topRightEu    = [-175, 35, 25]
topLeftEu     = [-160, 45, 35]
middleRightEu = [0, 90,  -45]
middleLeftEu  = [0, 90,  -30]
bottomRightEu = [0, 90,  30]
bottomLeftEu  = [0, 90, -30]

topRightPhi    = -45 
topLeftPhi     = -45
middleRightPhi = 40
middleLeftPhi  = 45
bottomRightPhi = 25
bottomLeftPhi  = 25

topRightSuc   = -68 
topLeftSuc    = -60


def setQuantity():
    for index in range(lunchQuan):
        objectName[index] = 'lunchboxXX'
        lunchboxPos[index][1] *= -1
        lunchboxPos[lunchQuan - index -1][2] += LUNCHBOX_H * index
    for index in range(4 - lunchQuan):
        lunchboxPos[4 - index -1][2] += LUNCHBOX_H * index
    for index in range(drinkQuan):
        objectName[index+4] = 'drinkXX'
    for index in range(riceQuan):
        objectName[index+8] = 'riceballXX'
    print lunchboxPos
    print objectName


class stockingTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        self.en_sim = False
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
        self.arm = ArmTask(self.name + '_arm')
        self.pickListAll = len(lunchboxPos) + len(riceballPos) + len(drinkPos)
        self.pickList = PICKORDER
        self.pos   = [0, 0, 0]
        self.euler = [0, 0, 0]
        self.phi   = 0
        self.sucAngle = 0
        if self.name == 'right':
            self.is_right = 1
            self.speed = SPEED_R
        if self.name == 'left':
            self.is_right = -1
            self.speed = SPEED_L
        if self.en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)
            rospy.on_shutdown(self.suction.gripper_vaccum_off)
        
    @property
    def finish(self):
        return self.pickList == self.pickListAll

    # def setQuantity(self):
    #     for index in range(lunchQuan):
    #         objectName[index] = 'lunchboxXX'
    #         lunchboxPos[index][1] *= -1
    #         lunchboxPos[lunchQuan - index -1][2] += LUNCHBOX_H * index
    #     for index in range(4 - lunchQuan):
    #         lunchboxPos[4 - index -1][2] += LUNCHBOX_H * index
    #         print LUNCHBOX_H * index
    #     for index in range(drinkQuan):
    #         objectName[index+4] = 'drinkXX'
    #     for index in range(riceQuan):
    #         objectName[index+8] = 'riceballXX'
    #     print lunchboxPos

    def getRearSafetyPos(self):
        self.pos   = [0, -0.5*self.is_right, -0.5]
        self.euler = [-90*self.is_right, -20, 30*self.is_right]
        self.phi   = -60*self.is_right

    def getFrontSafetyPos(self):
        self.pos   = (0.1, -0.4*self.is_right, -0.45)
        self.euler = (0, 0, 0*self.is_right)
        self.phi   = 45*self.is_right

    def getObjectPos(self):
        if self.finish:
            return
        while objectPos[self.pickList/4][self.pickList%4][1]*self.is_right > 0:
            self.pickList += 1 
            if self.finish:
                return
        self.pos   = objectPos[self.pickList/4][self.pickList%4][:]
        self.euler = objectEu[self.pickList/4][:]
        if objectName[self.pickList] == 'riceballXX':
            self.euler = riceballXXEu[:]
        self.euler[0] *= self.is_right
        self.euler[2] *= self.is_right
        self.phi   = -30*self.is_right
        if self.reGripCnt != 0:
            if self.reGripCnt == 1:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] += 0.01
            if self.reGripCnt == 2:
                if self.pickList == 4 or self.pickList == 6 or self.pickList == 8 or self.pickList == 10:
                    self.pos[0] += 0.005
                else:
                    self.pos[0] += 0.02
                self.pos[1] -= 0.01
            if self.reGripCnt == 3:
                self.pos[0] -= 0.01
                self.pos[1] -= 0.01

    def getPlacePos(self):
        if objectName[self.pickList] == 'lunchboxXX':
            self.pos   = bottomRight[:]
            self.euler = bottomRightEu[:]
            self.phi   = bottomRightPhi*self.is_right
            self.sucAngle = -90
            self.pos[2] += ((self.pickList%4))*0.05

        elif objectName[self.pickList] == 'lunchbox':
            self.pos   = bottomLeft[:]
            self.euler = bottomLeftEu[:]
            self.phi   = bottomLeftPhi*self.is_right
            self.sucAngle = -90
            self.pos[2] += ((self.pickList%4) - lunchQuan)*0.05

        elif objectName[self.pickList] == 'drinkXX':
            self.pos   = middleRight[:]
            self.euler = middleRightEu[:]
            self.phi   = middleRightPhi*self.is_right
            self.sucAngle = -90
            self.pos[0] += (drinkQuan - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'drink':
            self.pos   = middleLeft[:]
            self.euler = middleLeftEu[:]
            self.phi   = middleLeftPhi*self.is_right
            self.sucAngle = -90
            self.pos[0] += (4 - (self.pickList%4) - 1)*0.1

        elif objectName[self.pickList] == 'riceballXX':
            self.pos   = topLeft[:]
            self.euler = topLeftEu[:]
            self.phi   = topLeftPhi*self.is_right
            self.sucAngle = topLeftSuc
            self.pos[0] += (riceQuan - (self.pickList%4) - 1)*0.045

        elif objectName[self.pickList] == 'riceball':
            self.pos   = topRight[:]
            self.euler = topRightEu[:]
            self.phi   = topRightPhi*self.is_right
            self.sucAngle = topRightSuc
            self.pos[0] += (4 - (self.pickList%4) - 1)*0.045      

    def process(self):
        if self.arm.is_stop:                                       # must be include in your strategy
            self.finish = True                                     # must be include in your strategy
            print "!!! Robot is stop !!!"                          # must be include in your strategy
            self.suction.gripper_vaccum_off()                      # must be include in your strategy
            return                                                 # must be include in your strategy

        if self.state == idle:
            self.getObjectPos()
            if self.finish:
                return
            else:
                if 'riceball' in objectName[self.pickList] and self.pickList!=8:# or self.pickList==7:
                    self.state = safePose3
                    # self.state = rearSafetyPos
                else:
                    self.state = rearSafetyPos
                # self.state = rearSafetyPos
                print "self.pickList = " + str(self.pickList)
        
        elif self.state == busy:
            if self.arm.is_busy:
                if (self.nowState == leaveBin or self.nowState == frontSafetyPos or self.nowState == move2Shelf) and not self.suction.is_grip and not self.en_sim:
                    self.state = missObj
                return
            else:
                self.state = self.nextState
                self.nowState = self.nextState
                return

        elif self.state == safePose1:
            self.state = busy
            self.nextState = idle
            self.pickList += 1
            self.euler[2] = 90
            self.euler[0] = -10
            self.arm.relative_move('line', self.euler, [0, -0.1, -0.3], self.phi)

        elif self.state == leavePlacePos:
            self.state = busy
            self.nextState = leaveShelf
            if 'riceball' in objectName[self.pickList]:
                self.arm.relative_move('line', self.euler, [-0.02, 0, 0.04], self.phi)
            else:
                self.arm.noa_move_suction('line', suction_angle=self.sucAngle, n=0, o=0, a=-0.04)

        elif self.state == safePose3:
            self.state = busy
            self.nextState = rearSafetyPos
            self.arm.set_speed(self.speed)
            self.arm.jointMove(0, (0, -1.2, 0, 2.4, 0, -1.2, 0))

        elif self.state == safePose4:
            self.state = busy
            self.nextState = rearSafetyPos
            self.arm.set_speed(self.speed)
            self.arm.jointMove(0, (0, -1.2, 0, 2.4, 0, -1.2, 0))

        elif self.state == initPose:
            self.state = busy
            self.nextState = idle
            self.arm.set_speed(self.speed)
            self.arm.jointMove(0, (0, -1, 0, 1.57, 0, -0.57, 0))
            self.suction.gripper_suction_deg(0)


        elif self.state == frontSafetyPos:
            self.state = busy
            self.nextState = move2Shelf
            self.getRearSafetyPos()
            self.euler[0] = -90*self.is_right
            if 'drink' in objectName[self.pickList]:
                self.pos[2] = -0.4
                self.euler[1] = -25
            elif 'lunchbox' in objectName[self.pickList]:
                self.pos[2] = -0.45
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == rearSafetyPos:
            self.state = busy
            self.nextState = move2Bin
            self.getRearSafetyPos()
            # self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == rearSafetyPos2:
            self.state = busy
            self.nextState = move2Shelf
            self.getRearSafetyPos()
            self.euler[0] = -180
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == move2Bin:
            self.state = busy
            self.nextState = move2Object
            self.getObjectPos()
            self.pos[2] = -0.5
            if 'riceball' not in objectName[self.pickList]:
                self.euler[1] = -16
            else:
                self.euler[1] = -10
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
 
        elif self.state == move2Shelf:
            self.state = busy
            self.getPlacePos()
            if 'riceball' in objectName[self.pickList]:
                self.nextState = riceballEuler
                self.euler[0] = -45
                # self.euler = [0, -10, 0]
                # self.pos[2] -= 0.2
            else:
                self.nextState = moveIn2Shelf
                self.euler[0] = 0
            self.pos[0] = 0.42
            if 'lunchbox' in objectName[self.pickList]:
                self.pos[2] += 0.05
            else:
                self.pos[2] += 0.1
            self.arm.set_speed(self.speed)
            self.arm.noa_relative_pos('line', self.pos, self.euler, self.phi, suction_angle=0, n=0, o=0, a=-0.15)
            if 'riceball' not in objectName[self.pickList]:
                self.suction.gripper_suction_deg(-60)
                rospy.sleep(.1)
            self.suction.gripper_calibration()


        elif self.state == riceballEuler:
            self.state = busy
            self.nextState = moveIn2Shelf
            self.getPlacePos()
            self.pos[2] += 0.1
            self.arm.set_speed(self.speed)
            print 'euler = ', self.euler
            self.arm.move_euler('line', self.euler)
            self.suction.gripper_suction_deg(self.sucAngle)
        
        elif self.state == moveIn2Shelf:
            self.state = busy
            self.nextState = move2PlacedPos
            self.getPlacePos()
            if 'lunchbox' in objectName[self.pickList]:
                self.pos[2] += 0.05
            else:
                self.pos[2] += 0.1
            if self.pickList == 5:
                self.arm.set_speed(10)
            else:    
                self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            if 'riceball' not in objectName[self.pickList]:
                self.suction.gripper_suction_deg(-90)

        elif self.state == leaveBin:
            self.state = busy
            self.nextState = frontSafetyPos
            self.arm.set_speed(self.speed)
            self.getObjectPos()
            self.pos[2] = -0.47
            if 'drink' in objectName[self.pickList]:
                self.pos[0] -= 0.02
                self.pos[2] = -0.42
                self.euler[1] = -30
                self.euler[2] = 40*self.is_right
            if self.pickList == 10:
                self.euler[1] = -6
                self.euler[2] = 10*self.is_right
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == leaveShelf:
            self.state = busy
            self.nextState = idle
            self.arm.set_speed(self.speed)
            # if objectName[self.pickList] == 'riceballXX':
            #     self.arm.noa_move_suction('line', suction_angle=0, n=0.08, o=0, a=-0.22)
            # else:
            #     self.arm.noa_move_suction('line', suction_angle=0, n=0.08, o=0, a=-0.12)
            # self.arm.relative_move_pose('line', [-0.3, 0, 0.1])
            self.getPlacePos()
            if 'riceball' in objectName[self.pickList]:
                self.euler[0] = -45
                self.nextState = safePose1
                self.pickList -= 1
            else:
                self.euler[0] = 0
            self.pos[0] = 0.36
            self.pos[2] += 0.1
            self.arm.set_speed(self.speed)
            self.arm.noa_relative_pos('line', self.pos, self.euler, self.phi, suction_angle=0, n=0, o=0, a=-0.15)
            self.pickList += 1
            self.suction.gripper_suction_deg(0)

        elif self.state == move2Object:
            self.state = busy
            self.nextState = pickObject
            self.getObjectPos()
            self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == move2PlacedPos:
            self.state = busy
            self.nextState = placeObject
            self.getPlacePos()
            if self.pickList == 10 or self.pickList == 8:
                self.pos[2] -= 0.006
                if self.pickList == 8:
                    self.pos[2] += 0.003
            if 'lunchbox' in objectName[self.pickList]:
                self.arm.set_speed(80)
            else:    
                self.arm.set_speed(self.speed)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == pickObject:
            self.state = grasping
            self.suction.gripper_vaccum_on()
            # rospy.sleep(1)
            if 'lunchbox' in objectName[self.pickList]:
                self.arm.set_speed(30)
            else:
                self.arm.set_speed(3)
            self.arm.noa_move_suction('line', suction_angle=0, n=0, o=0, a=0.03)
            rospy.sleep(.1)
                        
        elif self.state == placeObject:
            self.state = busy
            self.nextState = leavePlacePos
            if 'lunchbox' in objectName[self.pickList]:
                self.nextState = leaveShelf
            # if 'riceball' in objectName[self.pickList]:
            #     self.arm.set_speed(80)
            #     self.arm.relative_move_pose('line', [-0.005, 0, 0])
            rospy.sleep(.3)
            self.suction.gripper_vaccum_off()

        elif self.state == grasping:
            if self.suction.is_grip or self.en_sim:
                self.arm.clear_cmd()
                # rospy.sleep(.1)
                self.state = busy
                self.nextState = leaveBin
                self.reGripCnt = 0
            elif not self.arm.is_busy:
                self.state = missObj
        
        elif self.state == missObj:
            if self.nowState == pickObject or self.nowState == leaveBin:
                self.state = busy
                self.nextState = move2Bin
                self.nowState = idle
                self.arm.clear_cmd()
                self.reGripCnt += 1
                if self.reGripCnt > 3:
                    self.reGripCnt = 0
                    self.pickList += 1
                    if self.finish:
                        self.nextState = idle
            elif self.nowState == frontSafetyPos or self.nowState == move2Shelf:
                self.state = busy
                self.nextState = idle
                self.nowState = idle
                self.arm.clear_cmd()
                self.pickList += 1 
                self.arm.set_speed(20)

def start_callback(msg):
    global is_start
    if msg.data == 1 and not is_start:
        is_start = True


if __name__ == '__main__':
    rospy.init_node('example')        # enable this node

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

    right = stockingTask('right')      # Set up right arm controller
    left  = stockingTask('left')       # Set up left arm controller
    rospy.sleep(.3)
    setQuantity()

    while not rospy.is_shutdown() and not is_start:
        rospy.loginfo('waiting for start signal')
        rospy.sleep(.5)
    
    SuctionTask.switch_mode(True)

    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown() and (not right.finish or not left.finish):
        left.process()
        right.process()
        rate.sleep()

    # robot arm back home
    if right.arm.is_stop is not True:
        rospy.loginfo('back home')
        left.arm.wait_busy()
        left.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))

        right.arm.wait_busy()
        right.arm.jointMove(0, (0, -1, 0, 2, 0, -0.7, 0))
    
        left.arm.wait_busy()
        left.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))

        right.arm.wait_busy()
        right.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))

    SuctionTask.switch_mode(False)
    # publish finish signal to wheels
    pub.publish(4)
