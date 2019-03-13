import os
import cv2 as cv

img = cv.imread("IMG_0847.JPG", 1)
#cv.imshow("image", img)
resize_img = cv.resize(img, (416, 416))
cv.imwrite("defect_.jpg", resize_img)
num = num + 1

cv.destroyAllWindows()
