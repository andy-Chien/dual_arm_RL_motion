#!/usr/bin/env python

from mobile_platform.srv import *
from strategy.srv import *
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from arm_control import ArmTask, SuctionTask

import os
import sys
import rospy
import time

# SerialKey motion command number
nIDEL               = 0
nMoveToP1           = 1
nMoveToP2           = 2
nRotToDeg90         = 3
nRotToDeg0          = 4
nTakeObj_Ori        = 5
nTakeObj_MoveDown   = 6
nTakeObj_BesideDrink= 7
nTakeObj_AboveMeal  = 8
nTakeObj_DrinkUp    = 9
nTakeObj_SuckDrink  = 10
nTakeObj_SuckMeal   = 11
nTakeObj_SuckMealKeepDown = 12
nTakeObj_SetSuckDrinkDeg  = 13
nTakeObj_TakeOut    = 14
nGiveObj1           = 15
nGiveObj2_AboveDesk = 16
nGiveObj2_OnDesk    = 17
nGiveObj2_LeaveDesk = 18
nDelaySuctOffObj1   = 19
nDelaySuctOffObj2   = 20
nInitArmPos         = 21
nIdelArmPos         = 22
nSTOP               = 23

# SerialKey motion command set
SerialKey_RobotIdel  = [nIDEL,      nSTOP]
SerialKey_LeadCustom = [nMoveToP1,  nSTOP]
SerialKey_TakeObjToCustom_Type1 = \
    [nRotToDeg90,           nIdelArmPos,        nTakeObj_Ori,       nTakeObj_MoveDown,
     nTakeObj_BesideDrink,  nTakeObj_SuckDrink, nTakeObj_DrinkUp,   nTakeObj_SetSuckDrinkDeg,
     nTakeObj_TakeOut,      nRotToDeg0,         nGiveObj1,          nDelaySuctOffObj1,
     nIdelArmPos,           nInitArmPos,        nSTOP]
# SerialKey_TakeObjToCustom_Type2 = \
#     [nRotToDeg90,           nIdelArmPos,        nTakeObj_Ori,       nTakeObj_MoveDown,
#      nTakeObj_AboveMeal,    nTakeObj_SuckMeal,  nTakeObj_AboveMeal, nTakeObj_TakeOut,
#      nRotToDeg0,            nMoveToP2,          nGiveObj2_AboveDesk,nGiveObj2_OnDesk,
#      nDelaySuctOffObj2,     nGiveObj2_LeaveDesk,nIdelArmPos,        nInitArmPos,
#      nSTOP]
SerialKey_TakeObjToCustom_Type2 = \
    [nRotToDeg90,           nIdelArmPos,                nTakeObj_Ori,       nTakeObj_MoveDown,
     nTakeObj_AboveMeal,    nTakeObj_SuckMealKeepDown,  nTakeObj_AboveMeal, nTakeObj_TakeOut,
     nRotToDeg0,            nMoveToP2,                  nGiveObj2_AboveDesk,nGiveObj2_OnDesk,
     nDelaySuctOffObj2,     nGiveObj2_LeaveDesk,        nIdelArmPos,        nInitArmPos,
     nSTOP]
SerialKey_PaymentState = [nIDEL, nSTOP]

# SerialKey Num for function GetMissionSerialKey
RobotIdel  = 0
LeadCustom = 1
TakeObjToCustom_Type1 = 2
TakeObjToCustom_Type2 = 3
PaymentState = 4

class CDualArmTask:
    def __init__(self, _name = '/robotis'):
        """Initial object."""
        en_sim = False
        if len(sys.argv) >= 2:
            rospy.set_param('en_sim', sys.argv[1])
            en_sim = rospy.get_param('en_sim')

        self.name  = _name
        self.arm   = ArmTask(self.name + '_arm')
        self.pick_list = 2
        self.pos   = (0, 0, 0)
        self.euler = (0, 0, 0)
        self.phi   = 0
        if en_sim:
            self.suction = SuctionTask(self.name + '_gazebo')
            print "aa"
        else:
            self.suction = SuctionTask(self.name)
            print "bb"

    def SetSpeed(self, spd):
        self.arm.set_speed(spd)

    def IdelPos(self):
        self.arm.set_speed(50)
        self.arm.jointMove(0, (0, -0.5, 0, 1, 0, -0.5, 0))
        self.suction.gripper_suction_deg(0)

    def InitialPos(self):
        self.arm.set_speed(50)
        self.arm.jointMove(0, (0, 0, 0, 0, 0, 0, 0))
        self.suction.gripper_suction_deg(0)

    def MoveAbs(self, Line_PtP, Pos, Euler, Redun):
        if(Line_PtP == 'line'):
            Line_PtP = 'line'
        else:
            Line_PtP = 'p2p'
        self.arm.ikMove(Line_PtP, Pos, Euler, Redun)
    
    def MoveRelPos(self, Line_PtP, Pos):
        if(Line_PtP == 'line'):
            Line_PtP = 'line'
        else:
            Line_PtP = 'p2p'
        print('RelMove1')
        self.arm.relative_move_pose(Line_PtP, Pos)
        print('RelMove1')

    def SuctionEnable(self, On_Off):
        if(On_Off == True):
            self.suction.gripper_vaccum_on()
        elif(On_Off == False):
            self.suction.gripper_vaccum_off()

    def SetSuctionDeg(self, Deg):
        self.suction.gripper_suction_deg(Deg)

    def SuckSuccess(self):
        print('suckflag')
        return self.suction.is_grip

    def StopRobot_and_ClearCmd(self):
        # Force stop robot
        print('StopRobot')
        self.arm.clear_cmd()

    def PauseRobot(self, PauseFlag):
        # True : Pause
        # False: Continue
        self.arm.freeze(PauseFlag)
    
    def GetArmPos(self):
        print('Pos1')
        GetArmInfo = self.arm.get_fb()
        print('Pos2')
        ArmPos = GetArmInfo.group_pose.position
        print('Pos3')
        return ArmPos # .x .y .z

    def ChangeModeToGetSuckState(self, Flag):
        # True : Get suck success state
        # False: Arduino for RFID
        self.suction.switch_mode(Flag)

class CDualArmCommand(object):
    def __init__(self):
        self.right = CDualArmTask('right') # Set up right arm controller
        self.left  = CDualArmTask('left')  # Set up left arm controller

        self.DualArmIsBusyFlag = False

        self.LowSpd = 40
        self.HighSpd= 60

        self.SuckDrinkDeg   = 0
        self.SuckDrinkUpDeg = -45
        self.SuckMealDeg    = -89

        self.GiveObj1_DelayTime = 3 # sec
        self.GiveObj2_DelayTime = 2 # sec

        self.SuckMealKeepDownDirection = [0, 0, -0.083] # -0.06 m = -6 cm

        self.R_PosTakeObj_Ori        = [0.30,-0.3006, -0.46]
        self.R_OriTakeObj_Ori        = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]
        self.L_PosTakeObj_Ori        = [0.30, 0.3506, -0.46]
        self.L_OriTakeObj_Ori        = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]

        self.R_PosTakeObj_MoveDown   = [0.30,-0.3006, -0.56]
        self.R_OriTakeObj_MoveDown   = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]
        self.L_PosTakeObj_MoveDown   = [0.30, 0.3506, -0.56]
        self.L_OriTakeObj_MoveDown   = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]

        self.R_PosTakeObj_AboveMeal  = [0.52,-0.2476, -0.56]
        self.R_OriTakeObj_AboveMeal  = [0.00, 89.000,  0.00]
        self.L_PosTakeObj_AboveMeal  = [0.52, 0.3036, -0.56]
        self.L_OriTakeObj_AboveMeal  = [0.00, 89.000,  0.00]

        self.R_PosTakeObj_DrinkUp    = [0.45,-0.3006, -0.44]
        self.R_OriTakeObj_DrinkUp    = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]
        self.L_PosTakeObj_DrinkUp    = [0.45, 0.3506, -0.44]
        self.L_OriTakeObj_DrinkUp    = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]

        self.R_PosTakeObj_BesideDrink= [0.40,-0.3006,-0.585]
        self.R_OriTakeObj_BesideDrink= [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]
        self.L_PosTakeObj_BesideDrink= [0.40, 0.3506,-0.585]
        self.L_OriTakeObj_BesideDrink= [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]

        self.R_PosTakeObj_SuckMeal   = [0.52,-0.2476,-0.623] #[0.52, -0.2476, -0.64]
        self.R_OriTakeObj_SuckMeal   = [0.00, 89.000,  0.00]
        self.L_PosTakeObj_SuckMeal   = [0.52, 0.3036,-0.623] #[0.52,  0.3036, -0.64]
        self.L_OriTakeObj_SuckMeal   = [0.00, 89.000,  0.00]

        self.R_PosTakeObj_SuckDrink  = [0.45,-0.3006,-0.585]
        self.R_OriTakeObj_SuckDrink  = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036] 
        self.L_PosTakeObj_SuckDrink  = [0.45, 0.3506,-0.585]
        self.L_OriTakeObj_SuckDrink  = [0.00, 89.000,  0.00] #[5.029, 82.029, 4.036]

        self.R_PosTakeObj_TakeOut    = [0.15,-0.3006, -0.44]
        self.R_OriTakeObj_TakeOut    = [0.00, 89.000,  0.00]
        self.L_PosTakeObj_TakeOut    = [0.15, 0.3506, -0.44]
        self.L_OriTakeObj_TakeOut    = [0.00, 89.000,  0.00]

        self.R_PosGiveObj1           = [0.40,-0.3006, -0.40]
        self.R_OriGiveObj1           = [0.00, 89.000,  0.00]
        self.L_PosGiveObj1           = [0.40, 0.3506, -0.40]
        self.L_OriGiveObj1           = [0.00, 89.000,  0.00]

        self.R_PosGiveObj2_AboveDesk = [0.55,-0.3006, -0.45]
        self.R_OriGiveObj2_AboveDesk = [0.00, 89.000,  0.00]
        self.L_PosGiveObj2_AboveDesk = [0.55, 0.3506, -0.45]
        self.L_OriGiveObj2_AboveDesk = [0.00, 89.000,  0.00]

        self.R_PosGiveObj2_OnDesk    = [0.55,-0.3006, -0.52]
        self.R_OriGiveObj2_OnDesk    = [0.00, 89.000,  0.00]
        self.L_PosGiveObj2_OnDesk    = [0.55, 0.3506, -0.52]
        self.L_OriGiveObj2_OnDesk    = [0.00, 89.000,  0.00]
   
        self.R_PosGiveObj2_LeaveDesk = [0.25,-0.3006, -0.45]
        self.R_OriGiveObj2_LeaveDesk = [0.00, 89.000,  0.00]
        self.L_PosGiveObj2_LeaveDesk = [0.25, 0.3506, -0.45]
        self.L_OriGiveObj2_LeaveDesk = [0.00, 89.000,  0.00]

    def InitArmPos(self, select):                
        if(select == 'right'):
            self.right.InitialPos()     # Initial robot arm pose
        elif(select == 'left'):
            self.left.InitialPos()      # Initial robot arm pose
        else:
            self.right.InitialPos()     # Initial robot arm pose
            self.left.InitialPos()      # Initial robot arm pose

    def IdelArmPos(self, select):
        if(select == 'right'):
            self.right.IdelPos()     # Robot arm idel pose
        elif(select == 'left'):
            self.left.IdelPos()      # Robot arm idel pose
        else:
            self.right.IdelPos()     # Robot arm idel pose
            self.left.IdelPos()      # Robot arm idel pose

    def TakeObj_Ori(self, select):          # Take object orientation
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_Ori# [0.3, -0.3006, -0.46]
        R_Euler = self.R_OriTakeObj_Ori#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_Ori#[0.3, 0.3506, -0.46]
        L_Euler = self.L_OriTakeObj_Ori#[5.029, 82.029, 4.036]
        L_Redun = -60

        if(select == 'right'):
            self.right.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.HighSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.HighSpd)
            self.left.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def TakeObj_MoveDown(self, select):     # Move down for take object 
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_MoveDown#[0.3, -0.3006, -0.56]
        R_Euler = self.R_OriTakeObj_MoveDown#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_MoveDown#[0.3, 0.3506, -0.56]
        L_Euler = self.L_OriTakeObj_MoveDown#[5.029, 82.029, 4.036]
        L_Redun = -60
        
        if(select == 'right'):
            self.right.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.HighSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.HighSpd)
            self.left.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def TakeObj_AboveMeal(self, select):    # Above object (meal)
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_AboveMeal#[0.45, -0.3006, -0.56]
        R_Euler = self.R_OriTakeObj_AboveMeal#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_AboveMeal#[0.45, 0.3506, -0.56]
        L_Euler = self.L_OriTakeObj_AboveMeal#[5.029, 82.029, 4.036]
        L_Redun = -60
        
        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.SetSuctionDeg(self.SuckMealDeg)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.SetSuctionDeg(self.SuckMealDeg)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.SetSuctionDeg(self.SuckMealDeg)
            self.left.SetSuctionDeg(self.SuckMealDeg)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
    
    def TakeObj_DrinkUp(self, select):      # Above object 
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_DrinkUp#[0.45, -0.3006, -0.54]
        R_Euler = self.R_OriTakeObj_DrinkUp#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_DrinkUp#[0.45, 0.3506, -0.54]
        L_Euler = self.L_OriTakeObj_DrinkUp#[5.029, 82.029, 4.036]
        L_Redun = -60

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def TakeObj_BesideDrink(self, select):    # Beside object (drink)
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_BesideDrink#[0.40, -0.3006, -0.60]
        R_Euler = self.R_OriTakeObj_BesideDrink#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_BesideDrink#[0.40, 0.3506, -0.60]
        L_Euler = self.L_OriTakeObj_BesideDrink#[5.029, 82.029, 4.036]
        L_Redun = -60

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.SetSuctionDeg(self.SuckDrinkDeg)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.SetSuctionDeg(self.SuckDrinkDeg)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.SetSuctionDeg(self.SuckDrinkDeg)
            self.left.SetSuctionDeg(self.SuckDrinkDeg)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
            
    def TakeObj_SuckMeal(self, select):     # Take object and suck it (Meal)
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_SuckMeal#[0.45, -0.3006, -0.60]
        R_Euler = self.R_OriTakeObj_SuckMeal#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_SuckMeal#[0.45, 0.3506, -0.60]
        L_Euler = self.L_OriTakeObj_SuckMeal#[5.029, 82.029, 4.036]
        L_Redun = -60
        
        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.SuctionEnable(True)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.SuctionEnable(True)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.SuctionEnable(True)
            self.left.SuctionEnable(True)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def TakeObj_SuckMealKeepDown(self, select):
        # Keep move down to suck object until succsee or lower than limit_z
        if(select == 'right'):
            SelectArm = self.right
            SelectArm.SetSpeed(self.LowSpd)
            SelectArm.SuctionEnable(True)
            Limit_z = self.R_PosTakeObj_SuckMeal[2] # height of z
        elif(select == 'left'):
            SelectArm = self.left
            SelectArm.SetSpeed(self.LowSpd)
            SelectArm.SuctionEnable(True)
            Limit_z = self.L_PosTakeObj_SuckMeal[2] # height of z
        else:
            TakeObj_SuckMeal(select)

        # SelectArm.ChangeModeToGetSuckState(True)
        SelectArm.MoveRelPos('line', self.SuckMealKeepDownDirection)
        while((select == 'right') or (select == 'left')):
            if(SelectArm.SuckSuccess()):
                SelectArm.StopRobot_and_ClearCmd()
                time.sleep(0.1)
                break
        # SelectArm.ChangeModeToGetSuckState(False) 

    def TakeObj_SuckDrink(self, select):    # Take object and suck it (Drink)
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_SuckDrink#[0.45, -0.3006, -0.60]
        R_Euler = self.R_OriTakeObj_SuckDrink#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_SuckDrink#[0.45, 0.3506, -0.60]
        L_Euler = self.L_OriTakeObj_SuckDrink#[5.029, 82.029, 4.036]
        L_Redun = -60
        
        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)            
            self.right.SuctionEnable(True)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.SuctionEnable(True)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.SuctionEnable(True)
            self.left.SuctionEnable(True)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def TakeObj_SetSuckDrinkDeg(self, select):            
        if(select == 'right'):
            self.right.SetSuctionDeg(self.SuckDrinkUpDeg)
        elif(select == 'left'):
            self.left.SetSuctionDeg(self.SuckDrinkUpDeg)
        else:
            self.right.SetSuctionDeg(self.SuckDrinkUpDeg)
            self.left.SetSuctionDeg(self.SuckDrinkUpDeg)

    def TakeObj_TakeOut(self, select):      # Leave object region and take it
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosTakeObj_TakeOut#[0.15, -0.3006, -0.50]
        R_Euler = self.R_OriTakeObj_TakeOut#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosTakeObj_TakeOut#[0.15, 0.3506, -0.50]
        L_Euler = self.L_OriTakeObj_TakeOut#[5.029, 82.029, 4.036]
        L_Redun = -60

        if(select == 'right'):
            self.right.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.HighSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.HighSpd)
            self.left.SetSpeed(self.HighSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def GiveObj1(self, select):             # Give object to customer high
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosGiveObj1#[0.4, -0.3006, -0.40]
        R_Euler = self.R_OriGiveObj1#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosGiveObj1#[0.4, 0.3506, -0.40]
        L_Euler = self.L_OriGiveObj1#[5.029, 82.029, 4.036]
        L_Redun = -60

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def GiveObj2_AboveDesk(self, select):   # Give object above desk
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosGiveObj2_AboveDesk#[0.4, -0.3006, -0.50]
        R_Euler = self.R_OriGiveObj2_AboveDesk#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosGiveObj2_AboveDesk#[0.4, 0.3506, -0.50]
        L_Euler = self.L_OriGiveObj2_AboveDesk#[5.029, 82.029, 4.036]
        L_Redun = -60       

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def GiveObj2_OnDesk(self, select):      # Give object to desk
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosGiveObj2_OnDesk#[0.4, -0.3006, -0.55]
        R_Euler = self.R_OriGiveObj2_OnDesk#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosGiveObj2_OnDesk#[0.4, 0.3506, -0.55]
        L_Euler = self.L_OriGiveObj2_OnDesk#[5.029, 82.029, 4.036]
        L_Redun = -60       

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def GiveObj2_LeaveDesk(self, select):   # Leave desk
        # self.DualArmIsBusyFlag = True
        R_Pos   = self.R_PosGiveObj2_LeaveDesk#[0.25, -0.3006, -0.40]
        R_Euler = self.R_OriGiveObj2_LeaveDesk#[5.029, 82.029, 4.036]
        R_Redun = 60
        
        L_Pos   = self.L_PosGiveObj2_LeaveDesk#[0.25, 0.3506, -0.40]
        L_Euler = self.L_OriGiveObj2_LeaveDesk#[5.029, 82.029, 4.036]
        L_Redun = -60       

        if(select == 'right'):
            self.right.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
        elif(select == 'left'):
            self.left.SetSpeed(self.LowSpd)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)
        else:
            self.right.SetSpeed(self.LowSpd)
            self.left.SetSpeed(self.LowSpd)
            self.right.MoveAbs('line',R_Pos, R_Euler, R_Redun)
            self.left.MoveAbs('line',L_Pos, L_Euler, L_Redun)

    def DelaySuctOffObj1(self, select):
        time.sleep(self.GiveObj1_DelayTime)   # Delay for customer take object        
        if(select == 'right'):
            self.right.SuctionEnable(False)
        elif(select == 'left'):
            self.left.SuctionEnable(False)
        else:
            self.right.SuctionEnable(False)
            self.left.SuctionEnable(False)

    def DelaySuctOffObj2(self, select):
        time.sleep(self.GiveObj2_DelayTime)   # Delay for customer take object
        if(select == 'right'):
            self.right.SuctionEnable(False)
        elif(select == 'left'):
            self.left.SuctionEnable(False)
        else:
            self.right.SuctionEnable(False)
            self.left.SuctionEnable(False)

    def ChangeMode(self, select, ModeFlag):
        if(select == 'right'):
            self.right.ChangeModeToGetSuckState(ModeFlag)
        elif(select == 'left'):
            self.left.ChangeModeToGetSuckState(ModeFlag)

    def IDEL(self):
        # self.DualArmIsBusyFlag = False
        # Do nothing
        pass
    
    def DualArmIsBusy(self):
        self.DualArmIsBusyFlag = (self.right.arm.is_busy) or (self.left.arm.is_busy) 
        # self.DualArmIsBusyFlag = False # Force set flag for testing
        return self.DualArmIsBusyFlag

class CMobileCommand(object):
    def __init__(self):
        self.pub_behavior= rospy.Publisher('scan_black/strategy_behavior', Int32, queue_size = 1)
        #self.pub_start   = rospy.Publisher('scan_black/strategy_start', Bool, queue_size = 1)
        self.srv_start   = rospy.ServiceProxy('scan_black/strategy_start', strategy_start)
        rospy.Subscriber("scan_black/dualarm_start", Bool, self.Sub_DualArm_Start)

        self.MobileIsBusyFlag = False
        self.SendToSrvSuccessFlag = False

    def Mobile_START(self):
        # Move to point 1 (0 deg)
        self.MobileIsBusyFlag = True
        srvData = strategy_start()
        srvData.data = True
        #self.pub_start.publish(start)
        self.SendToSrvSuccessFlag = self.srv_start(srvData.data)
        print(self.SendToSrvSuccessFlag)

    def Mobile_AID(self):
        # Turn to abs 0 deg
        self.MobileIsBusyFlag = True
        behavior_type = Int32()
        behavior_type.data = 11
        self.pub_behavior.publish(behavior_type)

    def Mobile_ORDER(self):
        # Turn to abs +90 deg
        self.MobileIsBusyFlag = True
        behavior_type = Int32()
        behavior_type.data = 12
        self.pub_behavior.publish(behavior_type)

    def Mobile_NEXT(self):
        # Move to point 2 (0 deg)
        self.MobileIsBusyFlag = True
        behavior_type = Int32()
        behavior_type.data =  3
        self.pub_behavior.publish(behavior_type)

    def Sub_DualArm_Start(self, msg):
        # Subcriber of DualArmStart Callback funciton
        self.__DualArmFlag = msg.data
        if(self.__DualArmFlag == True):
            self.MobileIsBusyFlag = False

    def IDLE(self):
        self.MobileIsBusyFlag = False
        # Do nothing
        
    def MobileIsBusy(self):
        # self.MobileIsBusyFlag = False # Force set flag for testing
        return self.MobileIsBusyFlag

class CSubscribeConsume(object):
    def __init__(self):
        self.consume_msg = None
        rospy.Subscriber("/consume", String, self.Sub_ConsumeString)

    def Sub_ConsumeString(self, msg):
        # Subcriber of Consume Callback funciton
        self.__ConsumeString = msg.data
        self.consume_msg = self.__ConsumeString

    def Return_SubString(self):
        return self.consume_msg
    

def GetMissionSerialKey(MissionReq):
    if(MissionReq == RobotIdel or MissionReq == None):
        return SerialKey_RobotIdel
    elif(MissionReq == LeadCustom):
        return SerialKey_LeadCustom
    elif(MissionReq == TakeObjToCustom_Type1):
        return SerialKey_TakeObjToCustom_Type1
    elif(MissionReq == TakeObjToCustom_Type2):
        return SerialKey_TakeObjToCustom_Type2
    elif(MissionReq == PaymentState):
        return SerialKey_PaymentState
    else:
        return SerialKey_RobotIdel

def MotionKeyDetector(Key, MobileCommandSet, DualArmCommandSet, SelectArm):
    if(Key == nIDEL):
        print("IDEL")
        MobileCommandSet.IDLE()
        DualArmCommandSet.IDEL()       
    elif(Key == nMoveToP1):
        print("MoveToP1")
        MobileCommandSet.Mobile_START()
    elif(Key == nMoveToP2):
        print("MoveToP2")
        MobileCommandSet.Mobile_NEXT()
    elif(Key == nRotToDeg90):
        print("RotToDeg90")
        MobileCommandSet.Mobile_ORDER()
    elif(Key == nRotToDeg0):
        print("RotToDeg0")
        MobileCommandSet.Mobile_AID()

    elif(Key == nTakeObj_Ori):
        print("TakeObj_Ori")
        DualArmCommandSet.TakeObj_Ori(SelectArm)
    elif(Key == nTakeObj_MoveDown):
        print("TakeObj_MoveDown")
        DualArmCommandSet.TakeObj_MoveDown(SelectArm)
    elif(Key == nTakeObj_AboveMeal):
        print("TakeObj_AboveMeal")
        DualArmCommandSet.TakeObj_AboveMeal(SelectArm)
    elif(Key == nTakeObj_DrinkUp):
        print("TakeObj_DrinkUp")
        DualArmCommandSet.TakeObj_DrinkUp(SelectArm)
    elif(Key == nTakeObj_BesideDrink):
        print("TakeObj_BesideDrink")
        DualArmCommandSet.TakeObj_BesideDrink(SelectArm)
    elif(Key == nTakeObj_SuckDrink):
        print("TakeObj_SuckDrink")
        DualArmCommandSet.TakeObj_SuckDrink(SelectArm)
    elif(Key == nTakeObj_SuckMeal):
        print("TakeObj_SuckMeal")
        DualArmCommandSet.TakeObj_SuckMeal(SelectArm)
    elif(Key == nTakeObj_SuckMealKeepDown):
        print("TakeObj_SuckMealKeepDown")
        DualArmCommandSet.TakeObj_SuckMealKeepDown(SelectArm)
    elif(Key == nTakeObj_TakeOut):
        print("TakeObj_TakeOut")
        DualArmCommandSet.TakeObj_TakeOut(SelectArm)
    elif(Key == nTakeObj_SetSuckDrinkDeg):
        print("TakeObj_SetSuckDrinkDeg")
        DualArmCommandSet.TakeObj_SetSuckDrinkDeg(SelectArm) 

    elif(Key == nGiveObj1):
        print("GiveObj1")
        DualArmCommandSet.GiveObj1(SelectArm)
    elif(Key == nGiveObj2_AboveDesk):
        print("GiveObj2_AboveDesk")
        DualArmCommandSet.GiveObj2_AboveDesk(SelectArm)
    elif(Key == nGiveObj2_OnDesk):
        print("GiveObj2_OnDesk")
        DualArmCommandSet.GiveObj2_OnDesk(SelectArm)
    elif(Key == nGiveObj2_LeaveDesk):
        print("GiveObj2_LeaveDesk")
        DualArmCommandSet.GiveObj2_LeaveDesk(SelectArm)

    elif(Key == nDelaySuctOffObj1):
        print("DelaySuctOffObj1")
        DualArmCommandSet.DelaySuctOffObj1(SelectArm)
    elif(Key == nDelaySuctOffObj2):
        print("Delay_SuctOffObj2")
        DualArmCommandSet.DelaySuctOffObj2(SelectArm)
    elif(Key == nInitArmPos):
        print("InitArmPos")
        DualArmCommandSet.InitArmPos(SelectArm)
    elif(Key == nIdelArmPos):
        print("IdelArmPos")
        DualArmCommandSet.IdelArmPos(SelectArm)
    elif(Key == nSTOP):
        print("STOP")
        # Key in DualArm & Mobile Robot STOP function here.
        pass
    else:
        MobileCommandSet.IDLE()
        DualArmCommandSet.IDEL()  

def handle_state(req):
    try:
        Get_Req = req.state
        print("Returning [%s]"%(req.state)) # Show the state from Assistant
        
        if type(Get_Req) != int:
            # Input: Mission
            #   0  : Robot idel
            #   1  : Lead customer
            #   2  : Take object to customer type1
            #   3  : Take object to customer type2
            # Other: Robot idel
            raise NotImplementedError("Request state input illegal. Please input an integer.")

        MobileCommandSet = CMobileCommand()
        DualArmCommandSet= CDualArmCommand()

        SerialKeyIndex   = 0
        MissionExecuteFlag = True
        MotionSerialKey = GetMissionSerialKey(Get_Req)

        SelectArm = 'dual'
        if(Get_Req == TakeObjToCustom_Type1):
            SelectArm = 'left'  # Use right arm to take object 1
        elif(Get_Req == TakeObjToCustom_Type2):
            SelectArm = 'right' # Use left arm to take object 2

        while((MissionExecuteFlag == True) and (MotionSerialKey != None)):            
            if not(MobileCommandSet.MobileIsBusy() or DualArmCommandSet.DualArmIsBusy()):
                MotionKey = MotionSerialKey[SerialKeyIndex]

                if( (MotionKey == nMoveToP1)  or
                    (MotionKey == nMoveToP2)  or
                    (MotionKey == nRotToDeg90)or
                    (MotionKey == nRotToDeg0) or
                    (MotionKey == nSTOP)    ):
                    print("Set False Mode")
                    DualArmCommandSet.ChangeMode(SelectArm, False)
                else:
                    print("Set True Mode")
                    DualArmCommandSet.ChangeMode(SelectArm, True)

                MotionKeyDetector(MotionKey, MobileCommandSet, DualArmCommandSet, SelectArm)
                if(MotionKey != nSTOP):
                    if not ((MotionKey == nMoveToP1) and (MobileCommandSet.SendToSrvSuccessFlag == False)):
                        # Check the data send to service or not.
                        # if there were not, it would keep execute the motion (MoveToP1).
                        SerialKeyIndex += 1
                else:
                    SerialKeyIndex = 0
                    MissionExecuteFlag = False

    except Exception, exception:
        ResponseFlag = False
        ResponseInfo = exception.message

    except NotImplementedError, e:
        ResponseFlag = False
        ResponseInfo = e.message

    else:
        if(Get_Req == RobotIdel):
            ResponseFlag = False
            # ResponseInfo = "Mission: Robot idel finish."
            ResponseInfo = "Mission: Robot idel finish."

        elif(Get_Req == LeadCustom):
            ResponseFlag = True
            # ResponseInfo = "Mission: Lead customer finish."
            ResponseInfo = "We have arrived at Sprite's shelves, do you need anything else?"
        
        elif(Get_Req == TakeObjToCustom_Type1):
            ResponseFlag = True
            # ResponseInfo = "Mission: Take object to customer type1 finish."
            ResponseInfo = "Here you are."
        elif(Get_Req == TakeObjToCustom_Type2):
            ResponseFlag = True
            ResponseInfo = "Here are your meals"
        
        elif(Get_Req == PaymentState):
            SubConsumeString = CSubscribeConsume()
            PaymentInfo = "Payment process complete"
            TickTime_Begin = time.clock()
            print("State 4: Wait for pay")
            
            while(SubConsumeString.Return_SubString() != "consumeA"):
                # Wait 10 sec for payment
                TickTime_Now = time.clock()
                TotalTime = TickTime_Now - TickTime_Begin
                if(TotalTime >= 10):
                    PaymentInfo = "Payment failed"
                    break
            print("State 4: End wait for pay")
            ResponseFlag = True
            ResponseInfo = PaymentInfo

        else:
            ResponseFlag = False
            ResponseInfo = "????????"
    ### Response the result of Strategy
    res = AssistantStateResponse()
    res.success = ResponseFlag    # Bool  : An flag for Watson speaking or not.
    res.info    = ResponseInfo    # String: It's Watson contents of speaking when flag is true.
                                  # String: It's Watson mission commit when flag is false.>>>>>>> 95ab65846a43e08502ec6b3c4c43480b599a06cd
    return res

def assistant_server():
    rospy.init_node('assistant_node')
    s = rospy.Service('assistant_service', AssistantState, handle_state)
    print "Listening on TIMDA Assistant."
    rospy.spin()

if __name__ == "__main__":
    assistant_server()
    # Execute in the handle_state callback function