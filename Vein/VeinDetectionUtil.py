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


def getBinaryROIImg(roi):
    ''' Use image processing(smooth, thresholding and clustering) to segment the 
    veins from the skin '''
    ns = 5 # median filter blur window 
    blurred_ROI = skimage.filters.median(roi, selem=np.ones((ns, ns)))
    
    # TODO: make contrast ratio a parameter
    hi_contr_ROI = np.clip(1.1*blurred_ROI, 0, 255)
    vein_mask_thresh = 165
    # binary_ROI = blurred_ROI < vein_mask_thresh
    # binary_ROI = binary_ROI > 170
    binary_ROI = hi_contr_ROI < vein_mask_thresh

    # footprint = disk(0.5)
    # eroded = erosion(binary_ROI, footprint)

    # fig, [ax1, ax2] = plt.subplots(2)
    # ax1.imshow(hi_contr_ROI)
    # ax2.imshow(binary_ROI)
    # plt.plot()
    masked_ROI = blurred_ROI * binary_ROI 
    masked_ROI[masked_ROI != 0] = 100
    return masked_ROI

def get_vein_map(masked_ROI):
    ''' Use DBscan clustering to cluster the vein pixels'''
    indices = np.dstack(np.indices(masked_ROI.shape))
    indices_flat = np.reshape(indices, [-1,2])
    masked_ROI_flat = np.reshape(masked_ROI, [-1,1])
    xy_pixels = np.hstack((masked_ROI_flat, indices_flat))

    db = DBSCAN(eps=15, min_samples=200, metric = 'euclidean',algorithm ='auto').fit(xy_pixels) 
    core_coord = np.unravel_index(db.core_sample_indices_, masked_ROI.shape)
    cluster_map = img_as_float(np.zeros_like(masked_ROI))
    labels = db.labels_

    # detected clusters are represented by labels
    label_img = np.reshape(labels, masked_ROI.shape)

    return label_img


def get_vein(vein_map):
    ''' Return an arrays of pixels indices of the biggest vein on 
    the segmented vein_map 
    '''
    # TODO: make vein selection more robust 
    # vein_map == 1 is the biggest vein cluster (0 is always the skin background)
    # idx_array_tuple = np.where(vein_map==1)
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
    line_x = np.arange(0, 500)
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


def get_camera_to_robot_tf_matrix(camera_target_pt):
    # TODO: calculate transformation matrix
<<<<<<< HEAD
    dist_n2c = [35, 43, 280] # distance from needle tip point to camera origin 
=======
    dist_n2c = [0, 110, 280] # distance from needle tip point to camera origin 
>>>>>>> master
    Trans_n2c = np.asarray([[1,0,0,dist_n2c[0]], [0, 1, 0, dist_n2c[1]], [0, 0, 1, dist_n2c[2]], [0, 0, 0, 1]]) # translation from needle point to camera center 
    # Rot_y = np.asarray([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # 180 deg rotation around y axis 
    # Rot_z = np.asarray([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # 90 deg rotation around z axis, align needle axis directions with camera's 
    Trans_c2i = np.asarray([[1, 0, 0, camera_target_pt[0]], [0, 1, 0, camera_target_pt[1]], [0, 0, 1, camera_target_pt[2]], [0, 0, 0, 1]]) # translation from camera center to target insertion point
    # yaw_ang = camera_target_pt[3]
    # Rot_needle_yaw = np.asarray([[np.cos(yaw_ang), -np.sin(yaw_ang), 0],[np.sin(yaw_ang), np.cos(yaw_ang), 0],[0, 0, 1, 0],[0, 0, 0, 1]]) # needle rotation  

    # target_tf = Trans_n2c @ Rot_y @ Rot_z @ Trans_c2i 
    target_tf = Trans_n2c @ Trans_c2i 
    # print("target point coordinate from needle pt is: ", target_tf)
    print("The needle should move: \n" + str(np.around(target_tf[0,3], 1)) + "mm in X axis, \n" + str(np.around(target_tf[1, 3],1)) + "mm in Y axis")

    return target_tf 
    # return Trans_n2c @ Rot_y @ Rot_z @ Trans_c2i @ Rot_needle_yaw


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


def getTargetPoint2D(image, bbtlr, bbtlc, bbbrr, bbbrc):
    
    ROI_img = generateROIImg(image, bbtlr, bbtlc, bbbrr, bbbrc) 
    binary_roi = getBinaryROIImg(ROI_img)

    fig, axs = plt.subplots(2, 3)
    axs[0, 0].imshow(image, cmap=plt.cm.gray)
    axs[0, 0].set_title('original image')
    axs[0, 1].imshow(ROI_img, cmap=plt.cm.gray)
    axs[0, 1].set_title('selected ROI')
    axs[0, 2].imshow(binary_roi, cmap=plt.cm.gray)
    axs[0, 2].set_title('binary ROI')

    # plt.show()
    # vein_map = get_vein_map(ROI_img)
    vein_map = get_vein_map(binary_roi)
    vein_pxls = get_vein(vein_map)   
    # print(vein_pxls)
    # fig, (ax1, ax2) = plt.subplots(2)
    axs[1,0].imshow(vein_map)
    axs[1,0].set_title('segmented vein map')
    # print(vein_map.shape[0])
    vein_line_x, vein_line_y = get_center_vein_line(vein_pxls, vein_map.shape[0])
    # print("vein_line_x: ", vein_line_x)
    # print("vein_line_y: ", vein_line_y)

    axs[1,0].plot(vein_line_x, vein_line_y, '-k', label='Line model from all data')
    target_point = get_target_point_2d(vein_line_x, vein_line_y)
    # print("target_point: ", target_point)
    axs[1,0].plot(target_point[0], target_point[1], 'o', markersize=7)

    # draw circles around target point
    rgb_ROI_img = skimage.color.gray2rgb(ROI_img)
    axs[1,1].imshow(image, cmap=plt.cm.gray)
    pt = (target_point[0]+bbtlc, target_point[1]+bbtlr)
    axs[1,1].plot(pt[0], pt[1], 'o', markersize=7)
    axs[1,1].set_title('selected injection point')

    plt.show()

    # if input("enter \'n\' to return 0,0 ") == 'n':
        # return (0, 0)

    return pt


def getYawAngle(vec1, vec2):
    line_vec = [vein_line_x[1]-vein_line_x[0], vein_line_y[1]-vein_line_y[0]]
    x_axis_vec = [0, vein_line_y[1]-vein_line_y[0]]
    # print(line_vec)
    # print(x_axis_vec)

    yaw_angle = angle_between(line_vec, x_axis_vec) # in radians 
    # print("yaw angle: ", yaw_angle)

    return yaw_angle


if __name__ == "__main__":

    # load the image
    image = skimage.io.imread("arm_88.png")
    gray_image = skimage.color.rgb2gray(image)

    image_show = cv2.imread("arm_88.png")
    image_show = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # working region for current thresholding 
    mask_rty = 140
    mask_rtx = 500
    mask_lby = 670
    mask_lbx = 1000

    pt2d = getTargetPoint2D(gray_image, mask_rty, mask_rtx, mask_lby, mask_lbx)
    # pt3d = get_target_point_robot_pose(pt2d)


