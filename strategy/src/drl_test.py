#!/usr/bin/env python3

"""Use to generate arm task and run."""

import os
import sys
import rospy
import random
from arm_control import ArmTask, SuctionTask
from std_msgs.msg import String, Float64, Bool, Int32

idle            = 0
busy            = 1
initPose        = 2
startPos        = 3
testPos         = 4

class exampleTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        en_sim = False
        self.finish = False
        if len(sys.argv) >= 2:
            rospy.set_param('en_sim', sys.argv[1])
            en_sim = rospy.get_param('en_sim')
        self.name = _name
        self.state = initPose
        self.nextState = idle
        self.arm = ArmTask(self.name + '_arm')
        self.pick_list = 2
        self.pos   = (0, 0, 0)
        self.euler = (0, 0, 0)
        self.phi   = 0
        self.cnt = 0
        self.tmp_rr = -1
        if en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
        else:
            self.suction = SuctionTask(self.name)

    def getStartPos(self):
        if self.name == 'right':
            self.pos, self.euler, self.phi = (0.275, -0.35, -0.45), (90, 0, 0), 30
        elif self.name == 'left':
            self.pos, self.euler, self.phi = (0.275, 0.35, -0.45), (90, 0, 0),-30

    def getTestPos(self):
        rr = random.randrange(3)
        if rr == 0:
            self.pos = (0.2, 0.15, -0.45)
        elif rr == 1:
            self.pos = (0.35, 0.1, -0.45)
        elif rr == 2:
            self.pos = (0.2, -0.15, -0.45)
        elif rr == 3:
            self.pos = (0.35, -0.1, -0.45)
        self.euler, self.phi = (90, 0, 0), 0
 
    def proces(self):
        if self.arm.is_stop:                                       # must be include in your strategy
            self.finish = True                                     # must be include in your strategy
            print("!!! Robot is stop !!!")                         # must be include in your strategy
            return                                                 # must be include in your strategy

        if self.state == idle:
            if self.finish:
                return
            else:
                self.state = startPos

        elif self.state == initPose:
            self.state = busy
            self.nextState = idle
            if not self.cnt<10:
                self.finish = True
            self.arm.set_speed(50)
            self.arm.jointMove(0, (0, -0.5, 0, 1, 0, -0.5, 0))
        
        elif self.state == startPos:
            self.state = busy
            if self.cnt < 10:
                self.nextState = testPos
            else:
                self.nextState = initPose
            self.getStartPos()
            self.arm.set_speed(50)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)
            self.cnt += 1    

        elif self.state == testPos:
            self.state = busy
            self.nextState = startPos
            self.getTestPos()
            self.arm.set_speed(50)
            self.arm.ikMove('line', self.pos, self.euler, self.phi)

        elif self.state == busy:
            if self.arm.is_busy:
                return
            else:
                self.state = self.nextState
            
if __name__ == '__main__':
    rospy.init_node('example')        #enable this node
    right = exampleTask('right')      #Set up right arm controller
    left  = exampleTask('left')       #Set up left arm controller
    rospy.sleep(0.3)
    rate = rospy.Rate(10)  # 30hz
    while not rospy.is_shutdown() and (not right.finish or not left.finish):
        left.proces()
        right.proces()
        rate.sleep()
    print('well done')
   
