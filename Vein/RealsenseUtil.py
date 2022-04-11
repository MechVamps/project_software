import pyrealsense2 as rs
import numpy as np
import cv2
import time
import imutils


# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
#   "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
#   "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
#   "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
#   "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# Configure IR stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

# Start input streaming
pipeline_profile = pipeline.start(config)


device = pipeline_profile.get_device()
depth_sensor = device.query_sensors()[0]
emitter = depth_sensor.get_option(rs.option.emitter_enabled)
print("emitter = ", emitter)
set_emitter = 0
depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
print("new emitter = ", emitter1)

point = (0, 0)

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format("DICT_4X4_50"))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_50"])
arucoParams = cv2.aruco.DetectorParameters_create()


# Ignore first 1sec for camera warm-up
for i in range(30):
    frames = pipeline.wait_for_frames()


try:
    count = 0
    while True:

        # read ir image
        frames = pipeline.wait_for_frames()
        infrared_frame = frames.first(rs.stream.infrared)
        IR_image = np.asanyarray(infrared_frame.get_data())


        # read depth images
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue
        depth_image = np.asanyarray(depth_frame.get_data())
        
        # get depth of a point
        def show_distance(event, x, y, args, params):
            global point
            point = (x, y)
        
        # get depth distance of a mouse clicked point
        cv2.namedWindow("pointAndDetect")
        cv2.setMouseCallback("pointAndDetect", show_distance)
        cv2.circle(IR_image, point, 10, (0, 0, 255))
        distance = depth_frame.get_distance(point[0], point[1])
        # print("clicked at {}, {}".format(point[0],point[1]))
        cv2.putText(IR_image, "{}m".format(distance), (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

###################

        # detect ArUco markers in the input frame
        # detect_image = np.copy(IR_image)
        # detect_image = imutils.resize(detect_image, width=600)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(IR_image,
            arucoDict, parameters=arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(IR_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(IR_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(IR_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(IR_image, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(IR_image, (cX, cY), 4, (0, 0, 255), -1)

                # draw the ArUco marker ID on the frame
                cv2.putText(IR_image, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                qr_distance = depth_frame.get_distance(cX, cY)
                print("QR tag at {}, {}".format(cX, cY))
                cv2.putText(IR_image, "{}m".format(qr_distance), (cX, cY - 20), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

###################

        # Display image
        cv2.imshow('pointAndDetect', IR_image)
        
        # Save Camera Video Frames
        # time.sleep(1)
        # cv2.imwrite("fake_arm_" + str(count) + ".png", IR_image)

        # Exit on ESC key
        c = cv2.waitKey(1) % 0x100
        if c == 27:
            break

        count += 1

finally:
    pipeline.stop() 
    cv2.destroyAllWindows()


