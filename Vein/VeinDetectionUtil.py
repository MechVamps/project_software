import numpy as np
import glob
import cv2
import matplotlib.pyplot as plt
import skimage.io
import skimage.color
import skimage.filters
from skimage.draw import circle_perimeter
from skimage.transform import probabilistic_hough_line
from skimage.feature import canny
from skimage.morphology import disk  
from skimage.morphology import (erosion, dilation, opening, closing,  # noqa
                                white_tophat)
from skimage.util import img_as_float, img_as_ubyte
from skimage import measure
from skimage.measure import LineModelND, ransac
from sklearn.cluster import DBSCAN
import sys

# for debugging arrays 
np.set_printoptions(threshold=sys.maxsize)

# TODO: move to utilities 
def generateROIImg(img, r1, c1, r2, c2):
    ''' Crop an image of Region of Interest given the top left and bottom right 
    corner of a bounding box '''
    mask = np.zeros((r2-r1, c2-c1))
    for i in range(r1, r2):
        for j in range(c1, c2):
            mask[i-r1, j-c1] = img[i, j]
    return mask 


# TODO: test with better lighting and less noise again 
def get_image_from_stream():
    ''' Capture RGB image from realsense RGB stream '''
    return True  


def get_vein_map(img):
    ''' Use image processing(smooth, thresholding and clustering) to segment the 
    veins from the skin '''

    ns = 10 # median filter blur window 
    blurred_ROI = skimage.filters.median(img, selem=np.ones((ns, ns)))

    vein_mask_thresh = 160
    binary_ROI = blurred_ROI < vein_mask_thresh

    # apply DBscan clustering on selected ROI
    masked_ROI = blurred_ROI * binary_ROI 
    indices = np.dstack(np.indices(masked_ROI.shape))
    indices_flat = np.reshape(indices, [-1,2])
    masked_ROI_flat = np.reshape(masked_ROI, [-1,1])
    xy_pixels = np.hstack((masked_ROI_flat, indices_flat))

    db = DBSCAN(eps=10, min_samples=100, metric = 'euclidean',algorithm ='auto').fit(xy_pixels) 
    core_coord = np.unravel_index(db.core_sample_indices_, masked_ROI.shape)
    cluster_map = img_as_float(np.zeros_like(blurred_ROI))
    labels = db.labels_

    # detected clusters are represented by labels
    label_img = np.reshape(labels, masked_ROI.shape)

    return label_img


def get_vein(vein_map):
    ''' Return an arrays of pixels indices of the biggest vein on 
    the segmented vein_map 
    '''
    # vein_map == 1 is the biggest vein cluster (0 is always the skin background)
    idx_array_tuple = np.where(vein_map==1)
    x = np.array(idx_array_tuple[1])
    y = np.array(idx_array_tuple[0])
    idx_array = np.column_stack([x, y])

    return idx_array
 

def get_center_vein_line(vein, img_max_row):
    ''' Given target vein pixels, fit a center line in the selected vein '''
    model = LineModelND()
    model.estimate(vein)
    # generate coordinates of estimated models
    line_x = np.arange(0, 120)
    line_y = model.predict_y(line_x)

    # get the line within the image frame 
    y_img_ids = np.where(np.logical_and(line_y > 0, line_y < img_max_row)) 
    line_y = line_y[y_img_ids]
    line_x = line_x[y_img_ids]

    '''
    # robustly fit line only using inlier data with RANSAC algorithm
    model_robust, inliers = ransac(vein_pxls, LineModelND, min_samples=2,
                                   residual_threshold=1, max_trials=1000)
    line_y_robust = model_robust.predict_y(line_x)
    y_img_ids_r = np.where(np.logical_and(line_y_robust > 0, line_y_robust < 100)) 
    line_y_r = line_y_robust[y_img_ids_r]
    line_x_r = line_x[y_img_ids_r]
    '''
    return line_x, line_y


def get_target_point_2d(vein_cx, vein_cy):
    '''
    Given the center line of the target vein, select the center point of the line
    to be the target IV injection point in the camera frame 
    '''
    center_idx = len(vein_cx)//2 
    center_point = (int(vein_cx[center_idx]), int(vein_cy[center_idx]))

    return center_point


# def get_target_point_camera_pose(target_pt_2d):
#     # TODO: get depth from realsense 
#     return 0 


def get_target_point_robot_pose(camera_target_pt):
    # TODO: calculate transformation matrix
    R = [[-1,0,0],[0,-1,0],[0,0,1]]
    p = camera_target_pt
    T = [[R,p],[0,0,0,1]]
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def getTargetPoint2D(image, bbtlx, bbtly, bbbrx, bbbry):
    
    ROI_img = generateROIImg(image, bbtlx, bbtly, bbbrx, bbbry) 
    vein_map = get_vein_map(ROI_img)
    vein_pxls = get_vein(vein_map)   
    vein_line_x, vein_line_y = get_center_vein_line(vein_pxls, 100)
    print("vein_line_x: ", vein_line_x)
    print("vein_line_y: ", vein_line_y)
    target_point = get_target_point_2d(vein_line_x, vein_line_y)

    print("target_point: ", target_point)

    # draw circles around target point
    # rr, cc = circle_perimeter(target_point[1], target_point[0], 5)
    rr, cc = circle_perimeter(target_point[1] + bbtlx, target_point[0] + bbtly, 15)
    
    # rgb_ROI_img = skimage.color.gray2rgb(ROI_img)
    print(rr, cc)
    # image_show[rr, cc] = (255, 0, 0)
    # rgb_ROI_img[rr, cc] = (255, 0, 0)

    fig, (ax1, ax2) = plt.subplots(2)
    ax1.imshow(vein_map, cmap=plt.cm.gray)
    ax2.imshow(vein_map, cmap=plt.cm.gray)
    ax2.plot(vein_line_x, vein_line_y, '-k', label='Line model from all data')
    plt.show()

    return (rr, cc)


def getYawAngle(vec1, vec2):
    line_vec = [vein_line_x[1]-vein_line_x[0], vein_line_y[1]-vein_line_y[0]]
    x_axis_vec = [0, vein_line_y[1]-vein_line_y[0]]
    # print(line_vec)
    # print(x_axis_vec)

    yaw_angle = angle_between(line_vec, x_axis_vec) # in radians 
    print("yaw angle: ", yaw_angle)

    return yaw_angle


if __name__ == "__main__":

    # load the image
    image = skimage.io.imread("fake_arm_0.png")
    gray_image = skimage.color.rgb2gray(image)

    image_show = cv2.imread("fake_arm_0.png")
    image_show = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # working region for current thresholding 
    mask_rtx = 200
    mask_rty = 280
    mask_lbx = 300
    mask_lby = 400

    getTargetPoint2D(gray_image, mask_rtx, mask_rty, mask_lbx, mask_lby)


# # Number of clusters in labels, ignoring noise if present.
# n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
# n_noise_ = list(labels).count(-1)

# print("Estimated number of clusters: %d" % n_clusters_)
# print("Estimated number of noise points: %d" % n_noise_)


# Display the image and plot all contours found
# fig, ([[ax1, ax2, ax3], [ax4, ax5, ax6]]) = plt.subplots(2, 3)
# ax1.imshow(image_show)
# ax2.imshow(gray_image, cmap=plt.cm.gray)
# # ax2.imshow(blurred_ROI, cmap=plt.cm.gray)
# # ax3.imshow(binary_ROI, cmap=plt.cm.gray)
# ax4.imshow(vein_map, cmap=plt.cm.gray)
# ax5.imshow(vein_map, cmap=plt.cm.gray)
# ax5.plot(vein_line_x, vein_line_y, '-k', label='Line model from all data')
# plt.show()
# ax6.plot(line_x_r, line_y_r, '-k', label='Line model from all data')
# ax6.imshow(rgb_ROI_img, cmap=plt.cm.gray)

'''
# draw circles around insertion point
selected_point = insert_candidates[0]
rr, cc = circle_perimeter(selected_point[0], selected_point[1], 3)
# selection = img_as_ubyte(selection)
cand_points[rr, cc] = 1

#show greyscale histogram 
histogram, bin_edges = np.histogram(blurred_image, bins=(mask_lbx-mask_rtx)*(mask_lby-mask_rty), range=(0, 256))
plt.plot(bin_edges[0:-1], histogram)
plt.title("Grayscale Histogram")
plt.xlabel("grayscale value")
plt.ylabel("pixels")
plt.xlim(0, 256)
plt.show()

# physcial reachable region 
mask_rtr = 250
mask_rtc = 280
mask_lbr = 380
mask_lbc = 480

ROI_img = generateROIImg(gray_image, mask_rtr, mask_rtc, mask_lbr, mask_lbc) 
'''


