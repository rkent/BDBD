## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

intrinsics = None
depth_scale = None

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

context = rs.context()
sensors = context.query_all_sensors()
print(sensors)

for sensor in sensors:
    if sensor.is_depth_sensor():
        ds = sensor.as_depth_sensor()
        depth_scale = ds.get_depth_scale()
        print("Depth scale: {}".format(depth_scale))

# setup filters
decimation = rs.decimation_filter()
temporal = rs.temporal_filter()
#temporal.set_option(rs.option.filter_smooth_alpha, .01)
print("temporal alpha", temporal.get_option(rs.option.filter_smooth_alpha))

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_frame = temporal.process(depth_frame)
        if intrinsics is None:
            profile = depth_frame.profile
            vprofile = profile.as_video_stream_profile()
            intrinsics = vprofile.get_intrinsics()
            print(intrinsics)

        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
            
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_meters_image = depth_image * depth_scale
        point_array = np.zeros([depth_image.shape[0], depth_image.shape[1], 3])
        print('point_array shape: {}'.format(point_array.shape))
        y_center = int(depth_image.shape[0]/2)
        x_center = int(depth_image.shape[1]/2)

        # calculate the point cloud
        # Note that pixels are (y, x, point) while point is (x, y, z)
        for i in range(int(depth_meters_image.shape[0])):
            for j in range(int(depth_meters_image.shape[1])):
                point = rs.rs2_deproject_pixel_to_point(
                    intrinsics, [float(j), float(i)], depth_meters_image[i, j])
                point_array[i, j, 0] = point[0] # x
                point_array[i, j, 1] = point[1] # y
                point_array[i, j, 2] = point[2] # z
                if j > 0 and i % 10 == 0 and point_array[i, j-1, 0] < 0. and point_array[i, j, 0 ] > 0.:
                    print('Center point ({:4.0f}, {:4.0f}): {:8.4f}, {:8.4f}, {:8.4f}'
                        .format(i, j, point_array[i, j, 0], point_array[i, j, 1], point_array[i, j, 2]))

        print('Center point: {:8.4f} {:8.4f} {:8.4f}'.format(
            point_array[y_center, x_center, 0], point_array[y_center, x_center,1], point_array[y_center, x_center,2]))
        '''

        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        #images = np.hstack((depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(5)
        # break
        '''

finally:

    # Stop streaming
    pipeline.stop()