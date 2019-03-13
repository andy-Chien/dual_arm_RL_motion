#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import sys
# sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import math
import numpy as np
import cv2

# lib 
from lib.nodehandle import NodeHandle
from lib.Image import Image

# rostopic 
from mobile_platform.msg import scaninfo


# FLAG
TIME_FLAG = False
DISERROR_FLAG = False
SCANSTATE_FLAG = False
IMGSHOW_FLAG = True
DIGITIAL_SIGINAL_FLAG = True
BLACK_FLAG = False
REMOVEBACKGROUND_FLAG = False

class ScanLine(object):
    def __init__(self):
        self._param = NodeHandle()
        self.images = None
        self._direction = 0
        self._multiPos = []
        self._angle = 999

        self.Init_Images()

    def Init_Images(self):
        self.images = []
        for q in range(self._param.sliceNum):
            self.images.append(Image(self._param.threshold,BLACK_FLAG))

    def Process(self,src):
        if(self._param.loadParam == True):
            self.Init_Images()
            self._param.loadParam = False
        
        self.SlicePart(src.copy())
        multipos = []
        row,col = src.shape[:2]
        for i in range(self._param.sliceNum):
            self._direction += self.images[i].dir
            multipos.append([self.images[i].pos[0],self.images[i].pos[1]+(row/float(self._param.sliceNum))*(i)])
        self._multiPos = multipos
        
        # print(self._multiPos)
        if(len(self._multiPos) != 0):
            self.Calculate_Angle()

        if(IMGSHOW_FLAG == True and self._param.imgShow):
            fm = self.RepackImages()
            cv2.imshow("Vision Race", fm)
    
    def SlicePart(self,src):
        height, width = src.shape[:2]
        sl = int(height/self._param.sliceNum)
        
        for i in range(self._param.sliceNum):
            part = sl*i
            crop_img = src[part:part+sl, 0:width]
            self.images[i].image = crop_img
            self.images[i].Process()

    def RepackImages(self):
        img = self.images[0].image
        for i in range(len(self.images)):
            if i == 0:
                img = np.concatenate((img, self.images[1].image), axis=0)
            if i > 1:
                img = np.concatenate((img, self.images[i].image), axis=0)
                
        return img

    def RemoveBackground(self,image, b):
        
        #----------------COLOR SELECTION-------------- (Remove any area that is whiter than 'upper')
        if b == True:
            if(BLACK_FLAG):
                up = 85
                # create NumPy arrays from the boundaries
                lower = np.array([0, 0, 0], dtype = "uint8")
                upper = np.array([up, up, up], dtype = "uint8")
                mask = cv2.inRange(image, lower, upper)

                image = cv2.bitwise_and(image, image, mask = mask)
                image = cv2.bitwise_not(image, image, mask = mask)
                image = (255-image)
            else:
                pass
            return image
        else:
            return image
    
    def Calculate_Angle(self):
        x = []
        y = []
        for i in range(len(self._multiPos)):
            if(self._multiPos[i][0] != 999):
                x.append(self._multiPos[i][0])
                y.append(self._multiPos[i][1])
        x = np.array(x)
        y = np.array(y)
        if(len(x)):
            xbar = np.sum(x)/float(len(x))
            ybar = np.sum(y)/float(len(y))

            self._angle = math.atan(np.sum((x-xbar)*(y-ybar))/float(np.sum((x-xbar)**2)))
        else:
            self._angle = 999
        # self._angle = (self._param.sliceNum*np.sum(x*y)-np.sum(x)*np.sum(y))/float(self._param.sliceNum*np.sum(x**2)-np.sum(x)**2)

        # print((self._param.sliceNum*np.sum(x*y)-np.sum(x)*np.sum(y)),float(self._param.sliceNum*np.sum(x**2)-np.sum(x)**2))        


    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, value):
        self._direction = value

    @property
    def multiPos(self):
        return self._multiPos

    @multiPos.setter
    def multiPos(self, value):
        self._multiPos = value
    
    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        self._angle = value


class DecideGrid(object):
    
    def __init__(self):
        self._param = NodeHandle()
        self.image = None
        self.scanImage = None
        self._scanState = None
    
    def Opening(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
        eroded = cv2.erode(self.thresh,kernel) 
        self.thresh = cv2.dilate(eroded,kernel)
    
    def Init_DecideGrid(self,src):
        row, col = src.shape[:2]
        if(self._param.middleY >= self._param.range):
            return src[self._param.middleY-self._param.range:self._param.middleY+self._param.range,0:col]
        else:
            return src[int(row/2)-self._param.range:int(row/2)+self._param.range,0:col]
    
    def Process(self,src):
        
        self.image = self.Init_DecideGrid(src.copy())
        
        # get mask 
        imgray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(imgray,(5,5),0)
        # ret,self.thresh = cv2.threshold(blur,self._threshold,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        if(BLACK_FLAG):
            ret,self.thresh = cv2.threshold(blur,self._param.threshold,255,cv2.THRESH_BINARY_INV)
        else:
            ret,self.thresh = cv2.threshold(blur,self._param.threshold,255,cv2.THRESH_BINARY)

        self.Opening()
        
        self.Slice_Scan()
        self.Detect_Scan()

        if(IMGSHOW_FLAG == True and self._param.imgShow):
            cv2.imshow("DecideGrid",self.thresh)

    def Draw_Scan(self,img):
        row,col = self.image.shape[:2]
        for i in range(self._param.scanNum):
            cv2.rectangle(img,(int(i*col/self._param.scanNum),self._param.middleY-self._param.range),(int((i+1)*col/self._param.scanNum),self._param.middleY+self._param.range),(0,255,255),3)
    
    def Slice_Scan(self):
        row,col = self.image.shape[:2]
        maskset = []
        for i in range(self._param.scanNum):
            maskset.append(self.thresh[0:row,int(i*col/self._param.scanNum):int((i+1)*col/self._param.scanNum)])
        self.scanImage = maskset
    
    def Detect_Scan(self):
        row,col = self.image.shape[:2]
        Num = float(row*col/self._param.scanNum)
        scanstate = []

        if(DIGITIAL_SIGINAL_FLAG):
            for mask in self.scanImage:
                if(cv2.countNonZero(mask) >= Num*(self._param.weight/100.0)):
                    scanstate.append(1)
                else:
                    scanstate.append(0)
        else:
            for mask in self.scanImage:
                scanstate.append(int(cv2.countNonZero(mask)/Num*100))
        self._scanState = scanstate
    
    @property
    def scanState(self):
        return self._scanState

    @scanState.setter
    def scanState(self, value):
        self._scanState = value

class ImageProcessing(object):
    def __init__(self):
        self._param = NodeHandle()
        self.scanLine = ScanLine()
        self.decideGrid = DecideGrid()
    
    def Process(self):
        if(self._param.img is not None):
            img = self.scanLine.RemoveBackground(self._param.img.copy(), REMOVEBACKGROUND_FLAG)
            
            if img is not None:
                self.scanLine.direction = 0
                self.scanLine.Process(img)
                
                if(DISERROR_FLAG == True):
                    print(self.scanLine.direction)

                self.decideGrid.Process(img)
                self.decideGrid.Draw_Scan(img)
                
                # if((self.scanLine.angle*180/math.pi)>0):
                #     print(-(self.scanLine.angle*180/math.pi)+90)
                # else:
                #     print(-(self.scanLine.angle*180/math.pi)-90)
                

                if(SCANSTATE_FLAG == True):
                    print(self.decideGrid.scanState)

                self.Pub_ScanInfo()

                if(IMGSHOW_FLAG == True and self._param.imgShow):
                    # cv2.imshow('src',src)
                    # cv2.imshow('dst',img)
                    cv2.waitKey(1)
                else:
                    cv2.destroyAllWindows()

    def Pub_ScanInfo(self):
        # if(self.decideGrid.scanState):
        #     info = scaninfo()
        #     info.dis = self.scanLine.direction
        #     info.scanstate = self.decideGrid.scanState
        # else:
        #     info = scaninfo()
        #     info.dis = 999
        #     info.scanstate = []
        
        info = scaninfo()
        info.dis = self.scanLine.direction
        if((self.scanLine.angle*180/math.pi)>0):
            info.angle = -(self.scanLine.angle*180/math.pi)+90
        else:
            info.angle = -(self.scanLine.angle*180/math.pi)-90
        info.scanstate = self.decideGrid.scanState
        self._param.pub_scaninfo.publish(info)
    

        
        
