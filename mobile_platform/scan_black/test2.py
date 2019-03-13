#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import sys
sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')

import numpy as np
import cv2

from Image import *
from Utils import *


# ======================================================
# Define

N_SLICES = 10

# ======================================================
# init

direction = 0
Images=[]

for q in range(N_SLICES):
    Images.append(Image())

# ======================================================

def main():
    cap = cv2.VideoCapture(0)

    while(True):
        ret, frame = cap.read()
        direction = 0
        img = RemoveBackground(frame.copy(), False)
        if img is not None:
            
            SlicePart(img.copy(), Images, N_SLICES)
            for i in range(N_SLICES):
                direction += Images[i].dir
            
            fm = RepackImages(Images)
            
            print(direction)
            cv2.imshow("Vision Race", fm)
            cv2.imshow('src',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()