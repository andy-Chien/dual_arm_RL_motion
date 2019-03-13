#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import sys
sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import numpy as np
import cv2

INNER_X = 100
INNER_Y = 30
OUTER_X = 150
OUTER_Y = 60
RANGE = 20

def main():
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        row,col,channel = frame.shape
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # ret,th3 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
        # blur = cv2.GaussianBlur(gray,(5,5),0)
        ret3,th3 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # th3 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,25,10)

        
        pos_ = []
        cnt = 0
        cnt_ = 0
        for y in range(int(row/2)-INNER_Y,int(row/2)+INNER_Y):
            pos1 = [0,0]
            pos2 = [0,0]
            for x in range(int(col/2)-INNER_X,int(col/2)+INNER_X):
                if(th3[y,x] == 0 and pos1[0] == 0):
                    pos1 = [x,y]
                    # print(pos1,pos2)
                elif(th3[y,x] == 0 and pos2[0] < x):
                    pos2 = [x,y]     
            if(pos2[0]-pos1[0] >= RANGE):
                pos_.append([pos1,pos2])
                cnt_ += 1
                if(cnt < cnt_):
                    cnt = cnt_
                    pos = pos_
            else:
                cnt_ = 0
        pos = np.array(pos)

        if(cnt > 5):
            if(((pos[int(cnt/2)][0][0]+pos[int(cnt/2)][1][0])/2) > (col/2)+3):
                print('too right')
            elif(((pos[int(cnt/2)][0][0]+pos[int(cnt/2)][1][0])/2) < (col/2)-3):
                print('too left')
            else:
                print('middle')
        else:
            print('Not Found')       

        # if(((pos[0][0]+pos[1][0])/2) > col/2 and cnt > 5):
        #     print('too right')
        # elif(((pos[0][0]+pos[1][0])/2) < col/2 and cnt > 5):
        #     print('too left')
        # else:
        #     print('Not Found')
        # inner
        cv2.rectangle(frame,(int(col/2)-INNER_X,int(row/2)-INNER_Y),(int(col/2)+INNER_X,int(row/2)+INNER_Y),(0,255,0),3)
        # outer
        cv2.rectangle(frame,(int(col/2)-OUTER_X,int(row/2)-OUTER_Y),(int(col/2)+OUTER_X,int(row/2)+OUTER_Y),(255,0,0),3)
        if(cnt > 5):
            cv2.circle(frame,tuple(pos[int(cnt/2)][0]), 3, (0,0,255), -1)
            cv2.circle(frame,tuple(pos[int(cnt/2)][1]), 3, (0,0,255), -1)
            
        # Display the resulting frame
        cv2.imshow('src',frame)
        cv2.imshow('threshold',th3)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()