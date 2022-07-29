## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

#json_path = 'HighResHighAccuracyPreset.json'

class camera():
    def __init__(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        #advanced_mode = rs.rs400_advanced_mode(device)
        #with open(json_path, 'r') as file:
        #    json = file.read().strip()
        #    advanced_mode.load_json(json)

        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 15)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

        # Start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        # print("Depth Scale is: ", depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 2.0 # 1 meter
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        # self.depth_image = np.zeros((480, 640), float)
        # self.color_image = np.zeros((480, 640), int)
        self.depth_image = np.zeros((720, 1280), float)
        self.color_image = np.zeros((720, 1280), int)


    def capture(self):
        frame_idx = 0
        save_num_counter = 0
        # Streaming loop
        while True:

            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            else:
                self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
                self.color_image = np.asanyarray(color_frame.get_data())
                break

            '''
            # cv2.imwrite('d415_output/ycb/depth_image_' + str(frame_idx)+'.png', depth_image)
            # cv2.imwrite('d415_output/ycb/color_image_' + str(frame_idx)+'.png', color_image)

            # frame_idx += 1
            # break
            
            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((self.depth_image,self.depth_image,self.depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, self.color_image)

            # Render images:
            #   depth align to color on left
            #   depth on right
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

            images = np.hstack((bg_removed, depth_colormap))

            cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
            cv2.imshow('Align Example', images)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            elif key & 0xFF == ord('s'):
                save_num_counter += 1
                cv2.imwrite('d435_output/depth_image_' + str(save_num_counter)+'.png', self.depth_image)
                cv2.imwrite('d435_output/color_image_' + str(save_num_counter)+'.png', self.color_image)
                depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
                depth_intrinsics = depth_profile.get_intrinsics()
                print(depth_intrinsics)
            # if frame_idx >= 300:
            #     break
            '''


    def __del__(self):
    # finally:
        self.pipeline.stop()
    # profile = pipeline.get_active_profile()
    # depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    # depth_intrinsics = depth_profile.get_intrinsics()
