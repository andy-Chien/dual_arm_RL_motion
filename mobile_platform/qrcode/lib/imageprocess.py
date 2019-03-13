#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import sys
# sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import math
import numpy as np
import cv2

# lib 
from lib.nodehandle import NodeHandle

# rostopic 
from geometry_msgs.msg import Pose2D

# FLAG
IMGSHOW_FLAG = True
QRANGLE_FLAG = False

class QRcode(object):
    def __init__(self):
        self._param = NodeHandle()
    
    def Process(self):
        if(self._param.img is not None):
            imgray = cv2.cvtColor(self._param.img.copy(),cv2.COLOR_BGR2GRAY)
            
            # 銳利化
            blur=cv2.GaussianBlur(imgray,(0,0),3)
            image=cv2.addWeighted(imgray,1.5,blur,-0.5,0)
            
            # 二值化
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
            ret,thresh = cv2.threshold(imgray,self._param.threshold,255,cv2.THRESH_BINARY)
            eroded = cv2.erode(thresh,kernel)
            thresh = cv2.dilate(eroded,kernel)
            # thresh = cv2.dilate(thresh,kernel)

            # 邊緣化
            if(self._param.cannyMin <= self._param.cannyMax):
                edges = cv2.Canny(thresh,self._param.cannyMin,self._param.cannyMax)
            else:
                edges = cv2.Canny(thresh,self._param.cannyMax,self._param.cannyMin)
            # kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
            # edges = cv2.dilate(edges,kernel)
            
            _, self.contours, self.hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
            
            if(IMGSHOW_FLAG and self._param.imgShow):
                cv2.imshow('thresh',thresh)
                cv2.imshow('edge',edges)
                cv2.waitKey(1)
            else:
                cv2.destroyAllWindows()
            if(self.contours):
                mark = 0
                mu = []
                mc = []
                A = 0
                B = 0
                C = 0
                median1 = 0
                median2 = 0
                outlier = 0
                for cnt in self.contours:
                    M,pos = self.getContourCenter(cnt)
                    mu.append(M)
                    mc.append(pos)

                for i in range(len(self.contours)):
                    peri = cv2.arcLength(self.contours[i], True)
                    approx = cv2.approxPolyDP(self.contours[i], 0.02 * peri, True)
                    if(len(approx) == 4):
                        k = i
                        c = 0
                        while(self.hierarchy[0][k][2] != -1):
                            k = self.hierarchy[0][k][2]
                            c = c+1
                        if(self.hierarchy[0][k][2] != -1):
                            c = c+1
                        if(c >= 5):
                            if(mark == 0):
                                A = i
                            elif(mark == 1):
                                B = i
                            elif(mark == 2):
                                C = i
                            mark += 1

                if(mark >= 3):
                    AB = self.PQ_Distance(mc[A],mc[B])
                    BC = self.PQ_Distance(mc[B],mc[C])
                    CA = self.PQ_Distance(mc[C],mc[A])

                    if(AB > BC and AB > CA):
                        median1 = A 
                        median2 = B
                        outlier = C
                    elif(CA > AB and CA > BC):
                        median1 = A
                        median2 = C
                        outlier = B
                    elif(BC > AB and BC > CA):
                        median1 = B 
                        median2 = C
                        outlier = A
                    dist = self.Line_Equation(mc[median1],mc[median2],mc[outlier])
                    slope = self.Line_Slope(mc[median1],mc[median2])
                    center_vec = self.Calculator_Center_Vector(mc[median1],mc[median2])
                    # print(center_dis)
                    self.Pub_Angle(True,dist,slope,center_vec)
                    
                    if(QRANGLE_FLAG):
                        if(slope < 0 and dist < 0):
                            print(1,(math.atan(slope)*180/3.14)+45)
                        elif(slope > 0 and dist < 0):
                            print(2,(math.atan(slope)*180/3.14)+45)
                        elif(slope < 0 and dist > 0):
                            if((math.atan(slope)*180/3.14) > -45):
                                print(3,(math.atan(slope)*180/3.14)-135)
                            else:
                                print(3,(math.atan(slope)*180/3.14)+225)
                        elif(slope > 0 and dist > 0):
                            print(4,(math.atan(slope)*180/3.14)-135)
                    
                    if(IMGSHOW_FLAG and self._param.imgShow):
                        cv2.imshow('dst',self._param.img)
                    
                else:
                    pass
                    print('Mark too low')
                    # self.Pub_Angle(False,0,0)
            else:
                print('Not Found')
                self.Pub_Angle(False,0,0,(0,0))
                    

            
    def getContourCenter(self, contour):
        M = cv2.moments(contour)
        
        if M["m00"] == 0:
            return 0,[0.0,0.0]
        
        x = float(M["m10"]/M["m00"])
        y = float(M["m01"]/M["m00"])
        
        return M,[x,y]

    def PQ_Distance(self,P,Q):
        return math.sqrt(pow(P[0]-Q[0],2.0)+pow(P[1]-Q[1],2.0))
    
    def Line_Equation(self,L,M,J):
        if(M[0]-L[0]):    
            a = -((M[1]-L[1])/(M[0]-L[0]))
            b = 1.0
            c = (((M[1]-L[1])/(M[0]-L[0]))*L[0]) - L[1]
            return (a*J[0]+b*J[1]+c)/math.sqrt((a*a)+(b*b))
        else:
            return 999

    def Line_Slope(self,L,M):
        dx = M[0]-L[0]
        dy = M[1]-L[1]

        if(dy != 0 and dx != 0):
            return (dy/dx)
        else:
            return 999

    def Calculator_Center_Vector(self,P,Q):
        center = (int((P[0]+Q[0])/2),int((P[1]+Q[1])/2))
        rows,cols = self._param.img.shape[:2]
        img_center = (int(cols/2),int(rows/2))
        cv2.circle(self._param.img,center, 5, (255,0,0), -1)
        cv2.circle(self._param.img,img_center, 5, (0,0,255), -1)

        # print(center[0]-img_center[0],center[1]-img_center[1]) 
        # return (center[0]-img_center[0],center[1]-img_center[1])
        return (img_center[0]-center[0],img_center[1]-center[1])     
        

        
    def Pub_Angle(self,find,dist,slope,vector):
        if(find):
            angle = (math.atan(slope)*180/3.14)
            if(slope < 0 and dist < 0):
                self._param.ang = angle+45
            elif(slope > 0 and dist < 0):
                self._param.ang = angle+45
            elif(slope < 0 and dist > 0):
                if(angle > -45):
                    self._param.ang = angle-135
                else:
                    self._param.ang = angle+225
            elif(slope > 0 and dist > 0):
                self._param.ang = angle-135
        else:
            self._param.ang = 999
        
        pose = Pose2D()
        pose.x = vector[0]
        pose.y = vector[1]
        pose.theta = self._param.ang
        self._param.pub_qrangle.publish(pose)
        
        

