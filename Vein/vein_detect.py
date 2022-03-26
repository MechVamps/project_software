import cv2
import numpy as np
import matplotlib
from matplotlib.pyplot import imshow
from matplotlib import pyplot as plt

im_gray = cv2.imread("fake_arm_19.png", cv2.IMREAD_GRAYSCALE)
_, mask = cv2.threshold(im_gray, thresh=180, maxval=255, type=cv2.THRESH_BINARY_INV)
im_thresh_gray = cv2.bitwise_and(im_gray, mask)

blur_image = cv2.medianBlur(im_gray,7)
cv2.imshow("blur",blur_image)

###
edge_img = cv2.Canny(blur_image,30,80)
cv2.imshow("edge_prefilter",edge_img)

# dilate
kernel = np.ones((3,3), np.uint8)
canned = cv2.dilate(edge_img, kernel, iterations = 2)
cv2.imshow("e2",canned)

# Display everything here
#cv2.imshow("im_gray", im_gray)
#cv2.imshow("im_thresh_gray", im_thresh_gray)
cv2.waitKey(0)