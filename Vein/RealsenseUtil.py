import pyrealsense2 as rs
import numpy as np
import cv2
import time
# import imutils
from VeinDetectionUtil import getTargetPoint2D, getYawAngle

class RealsenseUtil():
    def init(self):
        # Set ROI bounding box top left(bbtl[x/y]) and bottom right(bbbr[x/y]) corners 
        self.bbtlx = 200
        self.bbtly = 280
        self.bbbrx = 300
        self.bbbry = 400

    def get_target_point_camera_pose(self):
        target_point_camera_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Set ROI bounding box top left(bbtl[x/y]) and bottom right(bbbr[x/y]) corners 
        bbtlx = 200
        bbtly = 280
        bbbrx = 300
        bbbry = 400

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
            while True:
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
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', gray_image)
                cv2.waitKey(1)
                
                
                # infrared_frame = frames.first(rs.stream.infrared)
                # IR_image = np.array(infrared_frame.get_data())

                # # TODO: get target point from the detection pipeline
                cont = input()
                if cont == "y":
                    cv2.imwrite("arm_" + str(count) + ".png", gray_image)

                    # print("target_point: ", target_point)

                # target_point = getTargetPoint2D(color_image, bbtlx, bbtly, bbbrx, bbbry)
                depth = 9
                target_point = [187, 342]

                # align color and depth image

                align = rs.align(rs.stream.color)
                aligned_frames = align.proccess(frames)
                color_frame = aligned_frames.first(rs.stream.color)
                aligned_depth_frame = aligned_frames.get_depth_frame()

                # get the point in 3d      
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics           
                target_point_camera_pose = rs.rs2_deproject_pixel_to_point(
                                depth_intrin, target_point, depth)
                print(target_point_camera_pose)

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
                target_point_camera_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        finally:
            pipeline.stop() 
            cv2.destroyAllWindows()

        return target_point_camera_pose 

ru = RealsenseUtil()
ru.get_target_point_camera_pose()