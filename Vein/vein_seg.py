import numpy as np
import glob
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

# load the image
image = skimage.io.imread("fake_arm_0.png")
gray_image = skimage.color.rgb2gray(image)

def generateROIImg(img, x1, y1, x2, y2):
    mask = np.zeros((x2-x1, y2-y1))
    for i in range(x1, x2):
        for j in range(y1, y2):
            mask[i-x1, j-y1] = img[i, j]
    return mask 

mask_rtx = 200
mask_rty = 280
mask_lbx = 300
mask_lby = 400

ROI_img = generateROIImg(gray_image, mask_rtx, mask_rty, mask_lbx, mask_lby) 

ns = 10 # median filter blur window 
blurred_ROI = skimage.filters.median(ROI_img, selem=np.ones((ns, ns)))

# def ROI_histogram_bin(img, x1, y1, x2, y2):
#     bin = np.zeros((x2-x1, y2-y1))
#     for i in range(x1, x2):
#         for j in range(y1, y2):
#             bin[i-x1, j-y1] = img[i, j]
#     return bin

# ROI_box = ROI_histogram_bin(blurred_image, mask_rtx, mask_rty, mask_lbx, mask_lby)

## show ROI 
# fig, ax = plt.subplots()
# plt.imshow(ROI_box, cmap='gray')
# plt.show()

# histogram, bin_edges = np.histogram(blurred_image, bins=(mask_lbx-mask_rtx)*(mask_lby-mask_rty), range=(0, 256))
# show greyscale histogram 
# plt.plot(bin_edges[0:-1], histogram)
# plt.title("Grayscale Histogram")
# plt.xlabel("grayscale value")
# plt.ylabel("pixels")
# plt.xlim(0, 256)
# plt.show()


vein_mask_thresh = 160
binary_ROI = blurred_ROI < vein_mask_thresh

# footprint = disk(2)
# erosed_ROI = erosion(binary_ROI, footprint) # segmented veins 

# masked_ROI = np.zeros_like(blurred_ROI)
# masked_ROI = blurred_ROI*erosed_ROI
masked_ROI = blurred_ROI * binary_ROI 

indices = np.dstack(np.indices(masked_ROI.shape))
indices_flat = np.reshape(indices, [-1,2])
masked_ROI_flat = np.reshape(masked_ROI, [-1,1])
xy_pixels = np.hstack((masked_ROI_flat, indices_flat))

db = DBSCAN(eps=10, min_samples=100, metric = 'euclidean',algorithm ='auto').fit(xy_pixels) 
core_coord = np.unravel_index(db.core_sample_indices_, masked_ROI.shape)
cluster_map = img_as_float(np.zeros_like(blurred_ROI))
labels = db.labels_
label_img = np.reshape(labels, masked_ROI.shape)
cluster_map[label_img==0] = 0
cluster_map[label_img==1] = 0.5
cluster_map[label_img==2] = 1

# knowing label_img == 1 is the biggest cluster 
max_clus_indices = np.where(label_img==1)
x = np.array(max_clus_indices[1])
y = np.array(max_clus_indices[0])
clus_data = np.column_stack([x, y])
# clus_data = 1 
# print(clus_data)
# fit line using all data
model = LineModelND()
model.estimate(clus_data)
print(clus_data.shape)

# robustly fit line only using inlier data with RANSAC algorithm
model_robust, inliers = ransac(clus_data, LineModelND, min_samples=2,
                               residual_threshold=1, max_trials=1000)
outliers = inliers == False

# generate coordinates of estimated models
line_x = np.arange(0, 250)
line_y = model.predict_y(line_x)
line_y_robust = model_robust.predict_y(line_x)
print(line_x)
print(line_y)

line_vec = [line_x[1]-line_x[0], line_y[1]-line_y[0]]
x_axis_vec = [0, line_y[1]-line_y[0]]
print(line_vec)
print(x_axis_vec)

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

yaw_angle = angle_between(line_vec, x_axis_vec) # in radians 
print(yaw_angle)



# fig, ax = plt.subplots()
# ax.plot(clus_data[inliers, 0], clus_data[inliers, 1], '.b', alpha=0.6,
#         label='Inlier data')
# ax.plot(clus_data[outliers, 0], clus_data[outliers, 1], '.r', alpha=0.6,
#         label='Outlier data')
# ax.plot(line_x, line_y, '-k', label='Line model from all data')
# ax.plot(line_x, line_y_robust, '-b', label='Robust line model')
# ax.legend(loc='lower left')
# plt.show()


# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
n_noise_ = list(labels).count(-1)

print("Estimated number of clusters: %d" % n_clusters_)
print("Estimated number of noise points: %d" % n_noise_)


# Display the image and plot all contours found
fig, ([[ax1, ax2, ax3], [ax4, ax5, ax6]]) = plt.subplots(2, 3)
ax1.imshow(gray_image, cmap=plt.cm.gray)
ax2.imshow(blurred_ROI, cmap=plt.cm.gray)
ax3.imshow(binary_ROI, cmap=plt.cm.gray)
ax4.imshow(masked_ROI, cmap=plt.cm.gray)
ax5.imshow(cluster_map, cmap=plt.cm.gray)
ax6.imshow(cluster_map, cmap=plt.cm.gray)
ax6.plot(line_x, line_y, '-k', label='Line model from all data')

'''
# draw circles around insertion point
selected_point = insert_candidates[0]
rr, cc = circle_perimeter(selected_point[0], selected_point[1], 3)
# selection = img_as_ubyte(selection)
cand_points[rr, cc] = 1

# hough line detection 
edges = canny(selection, 3)
lines = probabilistic_hough_line(edges, threshold=5, line_length=20,
                                 line_gap=3)

# Find contours at a constant value of 0.8
contours = measure.find_contours(selection, 0.4)

fig, ax = plt.subplots()
for contour in contours:
    ax.plot(contour[:, 1], contour[:, 0], linewidth=2)

ax.axis('image')
ax.set_xticks([])
ax.set_yticks([])


for line in lines:
    p0, p1 = line
    ax3.plot((p0[0], p1[0]), (p0[1], p1[1]))
ax3.set_xlim((0, image.shape[1]))
ax3.set_ylim((image.shape[0], 0))

'''

plt.show()
