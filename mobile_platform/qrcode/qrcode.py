#!/usr/bin/env python
# -*- coding: utf-8 -*-+

# insert opencv 3.0 
import sys
# sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')


import roslib
roslib.load_manifest('mobile_platform')
import rospy

import numpy as np
import cv2

# lib 
from lib.imageprocess import QRcode

'''
    input: usb_cam/image_raw (type:sensor_msgs/Image)
    output: scan_black/qrcode_angle (type:std_msgs/Float32)

    口 口  =>  此為qrang的0度 
    口     =>
'''
 
def main():
    rospy.init_node('qrcode', anonymous=True)
    # cap = cv2.VideoCapture(0)
    image = QRcode()
    # 30 hz
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # if(cap.isOpened()):
        #     ret, frame = cap.read()
        #     if(frame is not None):
        #         image.Process(frame)
        #         cv2.imshow('src',frame)
        #         cv2.waitKey(1)
        if(image._param.start):
            image.Process()
            rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()