import pyrealsense2 as rs
import numpy as np
import cv2
import time
import imutils

# Configure IR stream
self.pipeline = rs.pipeline()
self.config = rs.config()
self.config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)
self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

# Start input streaming
self.pipeline_profile = pipeline.start(config)
self.device = pipeline_profile.get_device()
self.depth_sensor = device.query_sensors()[0]

self.emitter = depth_sensor.get_option(rs.option.emitter_enabled)
print("emitter = ", emitter)
self.set_emitter = 0
self.depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
self.emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
print("new emitter = ", emitter1)


# Ignore first 1sec for camera warm-up
for i in range(30):
    frames = pipeline.wait_for_frames()

try:
    # count = 0
    while True:

        # TODO: try to use the color frame

        # read ir image
        frames = pipeline.wait_for_frames()
        infrared_frame = frames.first(rs.stream.infrared)
        IR_image = np.asanyarray(infrared_frame.get_data())

        # TODO: get target point from the detection pipeline
        # target_point = getTargetPoint2D(IR_image)
        target_point = [0, 0]

        # read depth images
        depth_frame = frames.get_depth_frame()
        if not depth_frame: continue
        # depth_image = np.asanyarray(depth_frame.get_data())

        distance = depth_frame.get_distance(target_point[0], target_point[1])
        cv2.putText(IR_image, "{}m".format(distance), (target_point[0], target_point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

        # Display image
        cv2.imshow('pointAndDetect', IR_image)
        
        # Save Camera Video Frames
        # time.sleep(1)
        # cv2.imwrite("fake_arm_" + str(count) + ".png", IR_image)

        # Exit on ESC key
        c = cv2.waitKey(1) % 0x100
        if c == 27:
            break

        # count += 1

finally:
    pipeline.stop() 
    cv2.destroyAllWindows()


