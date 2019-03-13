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
from lib.imageprocess import ImageProcessing

'''
    input: usb_cam/image_raw (type:sensor_msgs/Image)
    output: scan_black/scaninfo (type:mobile_platform.msg/scaninfo)
        dis (error distance)
        angle (error angle)
        scanstate (模擬紅外線感測器)
'''
def main():
    rospy.init_node('detect_black2', anonymous=True)
    # cap = cv2.VideoCapture(0)
    image = ImageProcessing()
    # 30 hz
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # if(cap.isOpened()):
        #     ret, frame = cap.read()
        #     image.Process(frame)
        if(image._param.start):
            image.Process()
            rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()