import tensorflow as tf
import numpy as np
import math

class CheckCollision():
    def __init__(self):
        self.slave_start = tf.placeholder(tf.float32, [None, 3], 'slave_start')
        self.slave_end = tf.placeholder(tf.float32, [None, 3], 'slave_end')
        self.master_start = tf.placeholder(tf.float32, [None, 3], 'master_start')
        self.master_end = tf.placeholder(tf.float32, [None, 3], 'master_end')
        self.calculate_dis = self.calculateDistance(self.slave_start, self.slave_end, self.master_start, self.master_end)
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.1)
        self.sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
        self.sess.run(tf.global_variables_initializer())

    def checkCollision(self, RobotLinkPos_Master, RobotLinkPos_Slave):

        PointPosNum = len(RobotLinkPos_Slave[:,1])
        LinkNum = PointPosNum - 1
        Alarm = np.zeros(LinkNum)
        
        Slave2Master_Dist = np.ones((LinkNum, LinkNum-2), dtype=np.float32)*999
        for cnt_S in range(0,LinkNum):
            for cnt_M in range(2,LinkNum):
                vAB, vCD, vCA, vAC, vAD, v
                Slave2Master_Dist[cnt_S, cnt_M-2] = self.sess.run(self.calculate_dis, {self.slave_start: RobotLinkPos_Slave[cnt_S, :], self.slave_end: RobotLinkPos_Slave[cnt_S+1, :],
                                                                self.master_start: RobotLinkPos_Master[cnt_M, :], self.master_end: RobotLinkPos_Master[cnt_M+1, :]})                                                    
                if Slave2Master_Dist[cnt_S, cnt_M-2] < 0.16:
                    Alarm[cnt_S] = 1
        return Alarm

    def calculateDistance(self, pA, pB, pC, pD):
        vAB = pB - pA # u
        vCD = pD - pC # v
        vCA = pA - pC # r
        
        if tf.tensordot(tf.cross(vAB, vCD), vCA, axes=1) != 0: # check determination in 3-dimension or planar
            a = tf.tensordot( vAB, vAB, axes=1)
            b = tf.tensordot(-vAB, vCD, axes=1)
            c = tf.tensordot( vCD, vCD, axes=1)
            d = tf.tensordot( vAB, vCA, axes=1)
            e = tf.tensordot(-vCD, vCA, axes=1)
            f = tf.tensordot( vCA, vCA, axes=1)
            s = (b*e-c*d)/(a*c-b*b) # parameter in vector A (vector M, L1, AB, i-th robot link)
            t = (b*d-a*e)/(a*c-b*b) # parameter in vector B (vector N, L2, CD, j-th line obstacle)
            
            if (s < 0) and (t < 0):  # AC
                distance = tf.linalg.norm(vCA)
            elif (s < 0) and (0 <= t) and (t <= 1): # AC + tCD
                distance = tf.linalg.norm(-vCA + t*vCD)
            elif (s < 0) and (1 < t):  # AD
                distance = tf.linalg.norm(pD-pA)
            elif (0 <= s) and (s <= 1) and (t < 0): # CA + sAB
                distance = tf.linalg.norm(vCA + s*vAB)
            elif (0 <= s) and (s <= 1) and (0 <= t) and (t <= 1):
                distance = tf.sqrt(a*s**2 + 2*b*s*t + c*t**2 + 2*d*s + 2*e*t + f)
            elif (0 <= s) and (s <= 1) and (1 < t): # DA + sAB
                distance = tf.linalg.norm(pA-pD + s*vAB)           
            elif (1 < s) and (t < 0):  # BC
                distance = tf.linalg.norm(pC-pB)
            elif (1 < s) and (0 <= t) and (t <= 1): # BC + tCD
                distance = tf.linalg.norm(pC-pB + t*vCD)            
            elif (1 < s) and (1 < t):  # BD
                distance = tf.linalg.norm(pD-pB)
        else:
            vAC = pC - pA
            vAD = pD - pA
            vCB = pB - pC        
            s1 = tf.tensordot(vAC, vAB, axes=1)/(tf.linalg.norm(vAB)*tf.linalg.norm(vAB))
            s2 = tf.tensordot(vAD, vAB, axes=1)/(tf.linalg.norm(vAB)*tf.linalg.norm(vAB))
            t1 = tf.tensordot(vCA, vCD, axes=1)/(tf.linalg.norm(vCD)*tf.linalg.norm(vCD))
            t2 = tf.tensordot(vCB, vCD, axes=1)/(tf.linalg.norm(vCD)*tf.linalg.norm(vCD))   
            distance_planar = tf.constant([999, 999, 999, 999]) # s1 s2 t1 t2
            distance_Temp = 999
            
            if (0 <= s1) and (s1 <= 1):
                distance_planar[0] = tf.linalg.norm(vCA + s1*vAB)   # CA + s1*AB        
            if (0 <= s2) and (s2 <= 1):
                distance_planar[1] = tf.linalg.norm(-vAD + s2*vAB)  # DA + s2*AB        
            if (0 <= t1) and (t1 <= 1):
                distance_planar[2] = tf.linalg.norm(vAC + t1*vCD)   # AC + t1*CD        
            if (0 <= t2) and (t2 <= 1):
                distance_planar[3] = tf.linalg.norm(pC-pB + t2*vCD) # BC + t2*CD
            
            if (s1 < 0) and (s2 < 0) and (t1 < 0) and (t2 < 0):   # AC
                distance_planar[0] = tf.linalg.norm(vAC)
            elif (s1 > 1) and (s2 > 1) and (t1 < 0) and (t2 < 0): # BC
                distance_planar[1] = tf.linalg.norm(pC - pB)
            elif (s1 < 0) and (s2 < 0) and (t1 > 1) and (t2 > 1): # AD
                distance_planar[2] = tf.linalg.norm(vAD)
            elif (s1 > 1) and (s2 > 1) and (t1 > 1) and (t2 > 1): # BD
                distance_planar[3] = tf.linalg.norm(pD - pB)
            
            for cnt in range(1,4):
                if distance_Temp > distance_planar[cnt]:
                    distance_Temp =  distance_planar[cnt]
                    # cnt_Temp = cnt
            distance = distance_Temp
        return distance
