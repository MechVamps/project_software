import cv2
import numpy as np
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt

im_gray = cv2.imread("fake_arm_34.png", cv2.IMREAD_GRAYSCALE)
#_, mask = cv2.threshold(im_gray, thresh=180, maxval=255, type=cv2.THRESH_BINARY_INV)
#im_thresh_gray = cv2.bitwise_and(im_gray, mask)

# Blur original
blur_image = cv2.medianBlur(im_gray,7)
cv2.imshow("blur",blur_image)

# Mask to range (arm)
masked = cv2.inRange(im_gray, 135, 175)
res = cv2.bitwise_and(blur_image, masked)
cv2.imshow('real_filtered',res)

# Blur masked to remove filtered dots
blur_image2 = cv2.medianBlur(res,5)
cv2.imshow('blur2',blur_image2)

edge_img = cv2.Canny(blur_image2,100,200)
cv2.imshow("edges",edge_img)
cv2.waitKey(0)