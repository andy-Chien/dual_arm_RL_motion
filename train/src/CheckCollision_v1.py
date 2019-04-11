import numpy as np
import math

class CheckCollision():
    def __init__(self):
        self.threshold = np.array([[0.16, 0.16, 0.14],
                          [0.16, 0.14, 0.12], 
                          [0.14, 0.14, 0.12], 
                          [0.14, 0.14, 0.12], 
                          [0.12, 0.12, 0.1]])

    def checkCollision(self, RobotLinkPos_Master, RobotLinkPos_Slave):

        PointPosNum = len(RobotLinkPos_Slave[:,1])
        LinkNum = PointPosNum - 1
        Alarm = np.zeros(LinkNum)
        
        
        Slave2Master_Dist = np.ones((LinkNum, LinkNum-2), dtype=np.float32)*1.
        for cnt_S in range(0,LinkNum):
            for cnt_M in range(2,LinkNum):
                if cnt_S == 1 and cnt_M == 4:
                    RobotLinkPos_Slave[cnt_S, 1] = -1*RobotLinkPos_Slave[cnt_S+1, 1]
                Slave2Master_Dist[cnt_S, cnt_M-2] = self.calculateDistance( RobotLinkPos_Slave[cnt_S, :], RobotLinkPos_Slave[cnt_S+1, :],
                                                                        RobotLinkPos_Master[cnt_M, :], RobotLinkPos_Master[cnt_M+1, :])                                                    
                if Slave2Master_Dist[cnt_S, cnt_M-2] < self.threshold[cnt_S, cnt_M-2]:
                    Alarm[cnt_S] = 1
        Slave2Master_Dist = pow(self.threshold - Slave2Master_Dist + 1, 3)
        return Alarm,  np.reshape(Slave2Master_Dist, 15)

    def calculateDistance(self, pA, pB, pC, pD):
        vAB = pB - pA # u
        vCD = pD - pC # v
        vCA = pA - pC # r
        distance = 1.
        if math.fabs(np.dot(np.cross(vAB, vCD), vCA)) > 0.00000001: # check determination in 3-dimension or planar
            a = np.dot( vAB, vAB)
            b = np.dot(-vAB, vCD)
            c = np.dot( vCD, vCD)
            d = np.dot( vAB, vCA)
            e = np.dot(-vCD, vCA)
            f = np.dot( vCA, vCA)
            s = (b*e-c*d)/(a*c-b*b) # parameter in vector A (vector M, L1, AB, i-th robot link)
            t = (b*d-a*e)/(a*c-b*b) # parameter in vector B (vector N, L2, CD, j-th line obstacle)
            
            if (s < 0) and (t < 0):  # AC
                distance = np.linalg.norm(vCA)
            elif (s < 0) and (0 <= t) and (t <= 1): # AC + tCD
                distance = np.linalg.norm(-vCA + t*vCD)
            elif (s < 0) and (1 < t):  # AD
                distance = np.linalg.norm(pD-pA)
            elif (0 <= s) and (s <= 1) and (t < 0): # CA + sAB
                distance = np.linalg.norm(vCA + s*vAB)
            elif (0 <= s) and (s <= 1) and (0 <= t) and (t <= 1):
                distance = math.sqrt(a*s**2 + 2*b*s*t + c*t**2 + 2*d*s + 2*e*t + f)
            elif (0 <= s) and (s <= 1) and (1 < t): # DA + sAB
                distance = np.linalg.norm(pA-pD + s*vAB)           
            elif (1 < s) and (t < 0):  # BC
                distance = np.linalg.norm(pC-pB)
            elif (1 < s) and (0 <= t) and (t <= 1): # BC + tCD
                distance = np.linalg.norm(pC-pB + t*vCD)            
            elif (1 < s) and (1 < t):  # BD
                distance = np.linalg.norm(pD-pB)
            else:
                print('11111111111nodistancenodistancenodistancenodistancenodistancenodistancenodistance')
        else:
            vAC = pC - pA
            vAD = pD - pA
            vCB = pB - pC        
            s1 = np.dot(vAC, vAB)/(np.linalg.norm(vAB)*np.linalg.norm(vAB))
            s2 = np.dot(vAD, vAB)/(np.linalg.norm(vAB)*np.linalg.norm(vAB))
            t1 = np.dot(vCA, vCD)/(np.linalg.norm(vCD)*np.linalg.norm(vCD))
            t2 = np.dot(vCB, vCD)/(np.linalg.norm(vCD)*np.linalg.norm(vCD))   
            distance_planar = np.array([1., 1., 1., 1.]) # s1 s2 t1 t2
            distance_Temp = 1.
            
            if (0 <= s1) and (s1 <= 1):
                distance_planar[0] = np.linalg.norm(vCA + s1*vAB)   # CA + s1*AB        
            if (0 <= s2) and (s2 <= 1):
                distance_planar[1] = np.linalg.norm(-vAD + s2*vAB)  # DA + s2*AB        
            if (0 <= t1) and (t1 <= 1):
                distance_planar[2] = np.linalg.norm(vAC + t1*vCD)   # AC + t1*CD        
            if (0 <= t2) and (t2 <= 1):
                distance_planar[3] = np.linalg.norm(pC-pB + t2*vCD) # BC + t2*CD
            
            if (s1 < 0) and (s2 < 0) and (t1 < 0) and (t2 < 0):   # AC
                distance_planar[0] = np.linalg.norm(vAC)
            elif (s1 > 1) and (s2 > 1) and (t1 < 0) and (t2 < 0): # BC
                distance_planar[1] = np.linalg.norm(pC - pB)
            elif (s1 < 0) and (s2 < 0) and (t1 > 1) and (t2 > 1): # AD
                distance_planar[2] = np.linalg.norm(vAD)
            elif (s1 > 1) and (s2 > 1) and (t1 > 1) and (t2 > 1): # BD
                distance_planar[3] = np.linalg.norm(pD - pB)
            
            for cnt in range(4):
                if distance_Temp > distance_planar[cnt]:
                    distance_Temp =  distance_planar[cnt]
                    # cnt_Temp = cnt
            distance = distance_Temp
        return distance
