#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import math
import numpy as np
import matplotlib.pyplot as plt
import skfuzzy.control as ctrl

# lib
from lib.nodehandle import NodeHandle

# rostopic msg
from geometry_msgs.msg import Twist

# define behavior 
MOBILE_ROBOT = 0
CORRECTION = 1
PLATFORM = 2

# FLAG 

class FUZZYRule(object):
    def __init__(self):
        self.Init_Rule()
        self.System()
    
    def Init_Rule(self):
        universe0 = np.linspace(-3000, 3000, 5)      # Delta Destination Distance
        universe1 = np.linspace(-180, 180, 5)      # Delta Destination Angular
        universe2 = np.linspace(0, 25, 5)        # Velocity
        universe3 = np.array([-20, -15, 0, 15, 20]) # Angular Velocity

        dD = ctrl.Antecedent(universe0, 'dD')
        dT = ctrl.Antecedent(universe1, 'dT')
        oV = ctrl.Consequent(universe2, 'oV')
        oW = ctrl.Consequent(universe3, 'oW')

        # names0 = ['nb', 'ns',  'ze', 'ps', 'pb']
        dD.automf(names=['NHD', 'NLD',  'ZD', 'PLD', 'PHD'])
        dT.automf(names=['NHA', 'NLA',  'ZA', 'PLA', 'PHA'])
        oV.automf(names=['ZERO', 'SLOW',  'MEDIUM', 'FAST','HFAST'])
        oW.automf(names=['HL', 'L',  'ST', 'R', 'HR'])

        rule0 = ctrl.Rule(antecedent=(dD['NHD'] & dT['NHA']), consequent=(oV['SLOW'], oW['HR']))
        rule1 = ctrl.Rule(antecedent=(dD['NLD'] & dT['NHA']), consequent=(oV['SLOW'], oW['HR']))
        rule2 = ctrl.Rule(antecedent=(dD['ZD'] & dT['NHA']), consequent=(oV['SLOW'], oW['HR']))
        rule3 = ctrl.Rule(antecedent=(dD['PLD'] & dT['NHA']), consequent=(oV['SLOW'], oW['R']))
        rule4 = ctrl.Rule(antecedent=(dD['PHD'] & dT['NHA']), consequent=(oV['SLOW'], oW['ST']))

        rule5 = ctrl.Rule(antecedent=(dD['NHD'] & dT['NLA']), consequent=(oV['SLOW'], oW['HR']))
        rule6 = ctrl.Rule(antecedent=(dD['NLD'] & dT['NLA']), consequent=(oV['SLOW'], oW['R']))
        rule7 = ctrl.Rule(antecedent=(dD['ZD'] & dT['NLA']), consequent=(oV['MEDIUM'], oW['R']))
        rule8 = ctrl.Rule(antecedent=(dD['PLD'] & dT['NLA']), consequent=(oV['SLOW'], oW['ST']))
        rule9 = ctrl.Rule(antecedent=(dD['PHD'] & dT['NLA']), consequent=(oV['SLOW'], oW['L']))

        rule10 = ctrl.Rule(antecedent=(dD['NHD'] & dT['ZA']), consequent=(oV['MEDIUM'], oW['HR']))
        rule11 = ctrl.Rule(antecedent=(dD['NLD'] & dT['ZA']), consequent=(oV['MEDIUM'], oW['R']))
        rule12 = ctrl.Rule(antecedent=(dD['ZD'] & dT['ZA']), consequent=(oV['FAST'], oW['ST']))
        rule13 = ctrl.Rule(antecedent=(dD['PLD'] & dT['ZA']), consequent=(oV['MEDIUM'], oW['L']))
        rule14 = ctrl.Rule(antecedent=(dD['PHD'] & dT['ZA']), consequent=(oV['MEDIUM'], oW['HL']))

        rule15 = ctrl.Rule(antecedent=(dD['NHD'] & dT['PLA']), consequent=(oV['SLOW'], oW['R']))
        rule16 = ctrl.Rule(antecedent=(dD['NLD'] & dT['PLA']), consequent=(oV['SLOW'], oW['ST']))
        rule17 = ctrl.Rule(antecedent=(dD['ZD'] & dT['PLA']), consequent=(oV['MEDIUM'], oW['L']))
        rule18 = ctrl.Rule(antecedent=(dD['PLD'] & dT['PLA']), consequent=(oV['SLOW'], oW['L']))
        rule19 = ctrl.Rule(antecedent=(dD['PHD'] & dT['PLA']), consequent=(oV['SLOW'], oW['HL']))

        rule20 = ctrl.Rule(antecedent=(dD['NHD'] & dT['PHA']), consequent=(oV['SLOW'], oW['ST']))
        rule21 = ctrl.Rule(antecedent=(dD['NLD'] & dT['PHA']), consequent=(oV['SLOW'], oW['L']))
        rule22 = ctrl.Rule(antecedent=(dD['ZD'] & dT['PHA']), consequent=(oV['SLOW'], oW['HL']))
        rule23 = ctrl.Rule(antecedent=(dD['PLD'] & dT['PHA']), consequent=(oV['SLOW'], oW['HL']))
        rule24 = ctrl.Rule(antecedent=(dD['PHD'] & dT['PHA']), consequent=(oV['SLOW'], oW['HL']))

        self.my_rules = [rule0,  rule1,  rule2,  rule3,  rule4,
                    rule5,  rule6,  rule7,  rule8,  rule9,
                    rule10, rule11, rule12, rule13, rule14,
                    rule15, rule16, rule17, rule18, rule19,
                    rule20,  rule21,  rule22,  rule23,  rule24]

    def System(self):
        self.system = ctrl.ControlSystem(rules=self.my_rules)
        self.sim = ctrl.ControlSystemSimulation(self.system, flush_after_run=21 * 21 + 1)
    
    def Fuzzy_Calculate(self,dis,ang):
        if(dis is not None and ang is not None):
            self.sim.input['dD'] = dis
            self.sim.input['dT'] = -ang
            self.sim.compute()
        else:
            self.sim.input['dD'] = 0
            self.sim.input['dT'] = 0
            self.sim.compute()
        return(self.sim.output['oV'], self.sim.output['oW'])
    


class FUZZYControl(object):
    def __init__(self):
        self.fuzzy = FUZZYRule()

    def Process(self,dis,ang):
        vec,yaw= self.fuzzy.Fuzzy_Calculate(dis,ang)
        x = vec*math.cos(math.radians(ang))
        y = vec*math.sin(math.radians(ang))
        
        return x,y,yaw