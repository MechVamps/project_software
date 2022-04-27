import pyrealsense2 as rs
import numpy as np
import cv2
import time
# import imutils
from VeinDetectionUtil import getTargetPoint2D, getYawAngle, get_camera_to_robot_tf_matrix

def get_target_point_camera_pose():
    target_point_camera_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Set ROI bounding box top left(bbtl[x/y]) and bottom right(bbbr[x/y]) corners 
    mask_rty = 340
    mask_rtx = 550
    mask_lby = 520
    mask_lbx = 820

    # Configure IR stream
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1280, 720, rs.format.y8, 30)

    # Start input streaming
    pipeline_profile = pipeline.start(config)

    # Ignore first 1sec for camera warm-up
    for i in range(30):
        frames = pipeline.wait_for_frames()

    try:
        count = 0
        keep_detecting = True
        while keep_detecting:
            # read ir image
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', gray_image)
            # cv2.waitKey(1)
            
            
            # infrared_frame = frames.first(rs.stream.infrared)
            # IR_image = np.array(infrared_frame.get_data())

            # # TODO: get target point from the detection pipeline
            # if input("continue image processing") == "y":
            #     cv2.imwrite("arm_" + str(count) + ".png", gray_image)

            input("enter to cont.")
            target_point = getTargetPoint2D(gray_image, mask_rty, mask_rtx, mask_lby, mask_lbx)
            # print("target_point in image coord: ", target_point)
            depth = depth_frame.get_distance(target_point[0], target_point[1])
            # print("depth: ", depth)
            # target_point = [187, 342]

            # align color and depth image
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.first(rs.stream.color)
            aligned_depth_frame = aligned_frames.get_depth_frame()

            # get the point in 3d      
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics           
            target_point_camera_pose = rs.rs2_deproject_pixel_to_point(
                            depth_intrin, target_point, depth)
            target_point_camera_pose = np.dot(target_point_camera_pose, 1000) # m to mm
            # print(target_point_camera_pose)

            target_point_tf_matrix = get_camera_to_robot_tf_matrix(target_point_camera_pose)
            if input("enter \'y\' to start ik") == 'y':
                keep_detecting = False
            # # distance = depth_frame.get_distance(target_point[0], target_point[1])
            # # cv2.putText(IR_image, "{}m".format(distance), (target_point[0], target_point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

            
            # # Save Camera Video Frames
            # # time.sleep(1)
            # # cv2.imwrite("fake_arm_" + str(count) + ".png", IR_image)

            # # Exit on ESC key
            # c = cv2.waitKey(1) % 0x100
            # if c == 27:
            #     break

            count += 1
            # target_point_camera_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    finally:
        pipeline.stop() 
        # cv2.destroyAllWindows()

    return target_point_tf_matrix 

# ru = RealsenseUtil()
# ru.get_target_point_camera_pose()