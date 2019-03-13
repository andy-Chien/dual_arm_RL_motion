import numpy as np
import cv2

class Image:
    
    def __init__(self,threshold = 100,blackflag = True):
        self.image = None
        self.contourCenterX = 0
        self.MainContour = None
        self.dir = 0
        self.threshold = threshold
        self.blackflag = blackflag
        self.__pos = [999,999]
    
    def Opening(self,thresh):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3, 3))
        eroded = cv2.erode(thresh,kernel) 
        return cv2.dilate(eroded,kernel)
        
    def Process(self):
        imgray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY) #Convert to Gray Scale
        blur = cv2.GaussianBlur(imgray,(5,5),0)
        if(self.blackflag):
            ret,thresh = cv2.threshold(blur,self.threshold,255,cv2.THRESH_BINARY_INV)
        else:
            ret,thresh = cv2.threshold(blur,self.threshold,255,cv2.THRESH_BINARY)
        # blur = cv2.GaussianBlur(imgray,(5,5),0)
        # ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        thresh = self.Opening(thresh)

        _, self.contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) #Get contour
        
        if (self.contours):
            self.MainContour = max(self.contours, key=cv2.contourArea)
        
            self.height, self.width  = self.image.shape[:2]

            self.middleX = int(self.width/2) #Get X coordenate of the middle point
            self.middleY = int(self.height/2) #Get Y coordenate of the middle point
            
            self.prev_cX = self.contourCenterX
            if self.getContourCenter(self.MainContour) != 0:
                self.contourCenterX = self.getContourCenter(self.MainContour)[0]
                if abs(self.prev_cX-self.contourCenterX) > 5:
                    self.correctMainContour(self.prev_cX)
            else:
                self.contourCenterX = 0
            
            self.__pos = [self.contourCenterX-self.middleX,self.middleY]
            self.dir =  int((self.middleX-self.contourCenterX) * self.getContourExtent(self.MainContour))
            
            cv2.drawContours(self.image,self.MainContour,-1,(0,255,0),3) #Draw Contour GREEN
            cv2.circle(self.image, (self.contourCenterX, self.middleY), 7, (255,187,0), -1) #Draw dX circle BLUE
            cv2.circle(self.image, (self.middleX, self.middleY), 3, (0,0,255), -1) #Draw middle circle RED
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.image,str(self.middleX-self.contourCenterX),(self.contourCenterX+20, self.middleY), font, 1,(200,0,200),2,cv2.LINE_AA)
            cv2.putText(self.image,"Weight:%.3f"%self.getContourExtent(self.MainContour),(self.contourCenterX+20, self.middleY+35), font, 0.5,(200,0,200),1,cv2.LINE_AA)
        else:
            self.__pos = [999,999]
            self.dir = 0
            
            
        
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

    @property
    def pos(self):
        return self.__pos

    @pos.setter
    def pos(self, value):
        self.__pos = value                        
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            
                            