#!/usr/bin/env python
# -*- coding: utf-8 -*-+

import roslib
roslib.load_manifest('scan_black')
import rospy

# insert opencv 3.0 
import sys
# sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import numpy as np
import cv2

# rostopic 

from scan_black.msg import scaninfo
from std_msgs.msg import Int32


# FLAG
TIME_FLAG = False
DISERROR_FLAG = False
IMGSHOW_FLAG = True
DIGITIAL_SIGINAL_FLAG = True
BLACK_FLAG = True

class NodeHandle(object):
    def __init__(self):
        self._middleY = 240
        self._range = 30
        self._weight = 66
        self._threshold = 80
        self._scanNum = 7

        self.pub_scaninfo = rospy.Publisher('scan_black/scaninfo',scaninfo, queue_size = 1)

        self.sub_middleY = rospy.Subscriber("scan_black/middleY",Int32,self.Set_MiddleY)
        self.sub_range = rospy.Subscriber("scan_black/range",Int32,self.Set_Range)
        self.sub_weight = rospy.Subscriber("scan_black/weight",Int32,self.Set_Weight)
        self.sub_threshold = rospy.Subscriber("scan_black/threshold",Int32,self.Set_Threshold)
        self.sub_scanNum = rospy.Subscriber("scan_black/scanNum",Int32,self.Set_ScanNum)

    def Set_MiddleY(self,msg):
        self._middleY = msg.data
    def Set_Range(self,msg):
        self._range = msg.data
    def Set_Weight(self,msg):
        self._weight = msg.data
    def Set_Threshold(self,msg):
        self._threshold = msg.data
    def Set_ScanNum(self,msg):
        self._scanNum = msg.data 

class Image(NodeHandle):
    
    def __init__(self):
        super(Image,self).__init__()
        self.image = None
        self.maincontour = None
        self.contourCenterX = 0

        self.scan = None
        self.scan_state = None

    def Opening(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
        eroded = cv2.erode(self.thresh,kernel) 
        self.thresh = cv2.dilate(eroded,kernel)
        
        
    def Process(self):
        # get mask 
        imgray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(imgray,(5,5),0)
        # ret,self.thresh = cv2.threshold(blur,self._threshold,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        if(BLACK_FLAG):
            ret,self.thresh = cv2.threshold(blur,self._threshold,255,cv2.THRESH_BINARY_INV)
        else:
            ret,self.thresh = cv2.threshold(blur,self._threshold,255,cv2.THRESH_BINARY)

        self.Opening()

        # find contours
        _, self.contours, _ = cv2.findContours(self.thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
        
        if(self.contours):
            # get max contour
            self.maincontour = max(self.contours, key=cv2.contourArea)

            row,col,channel = self.image.shape
            self.middleX = int(col/2)
            self.middleY = int(row/2)

            # find contour center
            self.prev_cX = self.contourCenterX
            if self.getContourCenter(self.maincontour) != 0:
                self.contourCenterX = self.getContourCenter(self.maincontour)[0]
                x,y,w,h = cv2.boundingRect(self.maincontour)
                if abs(self.prev_cX-self.contourCenterX) > 5:
                    self.correctMainContour(self.prev_cX)
            else:
                self.contourCenterX = 0

            # distance error
            self.dir =  int((self.middleX-self.contourCenterX) * self.getContourExtent(self.maincontour))
            
            # scan info
            self.Slice_Scan()
            self.Detect_Scan()

            # decide state and info pub  
            self.Pub_ScanInfo(1)
            
            # Draw 
            cv2.drawContours(self.image, self.maincontour, -1, (0,255,0), 3)
            cv2.circle(self.image, (self.contourCenterX, self.middleY), 7, (255,255,255), -1) #Draw dX circle WHITE
            cv2.circle(self.image, (self.middleX, self.middleY), 7, (0,0,255), -1)
        else:
            self.Pub_ScanInfo(0)
        if(IMGSHOW_FLAG):
            cv2.imshow('mask',self.thresh)
            
 
    def getContourCenter(self, contour):
        M = cv2.moments(contour)
        
        if M["m00"] == 0:
            return 0
        
        x = int(M["m10"]/M["m00"])
        y = int(M["m01"]/M["m00"])
        
        return [x,y]
        
    def getContourExtent(self, contour):
        area = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour)
        rect_area = w*h
        if rect_area > 0:
            return (float(area)/rect_area)

    def Aprox(self, a, b, error):
        if abs(a - b) < error:
            return True
        else:
            return False
            
    def correctMainContour(self, prev_cx):
        if abs(prev_cx-self.contourCenterX) > 5:
            for i in range(len(self.contours)):
                if self.getContourCenter(self.contours[i]) != 0:
                    tmp_cx = self.getContourCenter(self.contours[i])[0]
                    if self.Aprox(tmp_cx, prev_cx, 5) == True:
                        self.MainContour = self.contours[i]
                        if self.getContourCenter(self.MainContour) != 0:
                            self.contourCenterX = self.getContourCenter(self.MainContour)[0]
    
    def Draw_Scan(self,img):
        row,col,channel = img.shape
        for i in range(self._scanNum):
            cv2.rectangle(img,(int(i*col/self._scanNum),self._middleY-self._range),(int((i+1)*col/self._scanNum),self._middleY+self._range),(0,255,255),3)
    
    def Slice_Scan(self):
        row,col = self.thresh.shape
        maskset = []
        for i in range(self._scanNum):
            maskset.append(self.thresh[0:row,int(i*col/self._scanNum):int((i+1)*col/self._scanNum)])
        self.scan = maskset

    def Detect_Scan(self):
        row,col,channel = self.image.shape
        Num = float(row*col/self._scanNum)
        scanstate = []

        if(DIGITIAL_SIGINAL_FLAG):
            for mask in self.scan:
                if(cv2.countNonZero(mask) >= Num*(self._weight/100.0)):
                    scanstate.append(1)
                else:
                    scanstate.append(0)
        else:
            for mask in self.scan:
                scanstate.append(int(cv2.countNonZero(mask)/Num*100))
        self.scan_state = scanstate
    
    def Pub_ScanInfo(self,find):
        if(find):
            info = scaninfo()
            info.dis = self.dir
            info.scanstate = self.scan_state
        else:
            info = scaninfo()
            info.dis = 999
            info.scanstate = []
        
        self.pub_scaninfo.publish(info)
        
        

def SlicePart(img,x,y,height):
    row,col,channel = img.shape
    return img[y-height:y+height,x-int(col/2):x+int(col/2)]

def main():
    
    rospy.init_node('detect_black', anonymous=True)
    cap = cv2.VideoCapture(0)
    imgs = Image()

    # 30 hz
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        if(TIME_FLAG):    
            e1 = cv2.getTickCount()

        if(cap.isOpened()):
            ret, frame = cap.read()
            row,col,channel = frame.shape
            
            if(imgs._middleY >= imgs._range):
                imgs.image = SlicePart(frame,int(col/2),imgs._middleY,imgs._range)
                imgs.Process()
                
                # outer
                # cv2.rectangle(frame,(0,imgs._middleY-imgs._range),(col,imgs._middleY+imgs._range),(255,0,0),3)
          
                imgs.Draw_Scan(frame)
            else:
                imgs.image = SlicePart(frame,int(col/2),int(row/2),imgs._range)
                imgs.Process()
                
                # outer
                # cv2.rectangle(frame,(0,int(row/2)-imgs._range),(col,int(row/2)+imgs._range),(255,0,0),3)
    
                imgs.Draw_Scan(frame)
            
            # show src
            if(IMGSHOW_FLAG):
                cv2.imshow('src',frame)
                cv2.waitKey(1)

            if(TIME_FLAG):
                e2 = cv2.getTickCount()
                time = (e2 - e1)/ cv2.getTickFrequency()
                print(time)

            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
        else:
            print('No camera')
            exit()
        # rospy.sleep(0.1)
        rate.sleep()  
    try:
        # pass
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()