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
filters = []
decimation = None

# decimation
#decimation = rs.decimation_filter()
#filters.append(decimation)

# temporal

temporal = rs.temporal_filter()
filters.append(temporal)
#temporal.set_option(rs.option.filter_smooth_alpha, .1)
print("temporal holes fill", temporal.get_option(rs.option.holes_fill))
#temporal.set_option(rs.option.holes_fill, 3)
print("temporal holes fill", temporal.get_option(rs.option.holes_fill))
print("temporal alpha", temporal.get_option(rs.option.filter_smooth_alpha))
print(temporal.get_supported_options())


# spatial
spatial = rs.spatial_filter()
print("spatial holes fill", spatial.get_option(rs.option.holes_fill))
spatial.set_option(rs.option.holes_fill, 2)
print("spatial holes fill", spatial.get_option(rs.option.holes_fill))
filters.append(spatial)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.array(depth_frame.get_data())

        filtered_depth_frame = depth_frame
        for filter in filters:
            filtered_depth_frame = filter.process(filtered_depth_frame)

        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        filtered_image = np.asanyarray(filtered_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if intrinsics is None:
            profile = depth_frame.profile
            vprofile = profile.as_video_stream_profile()
            intrinsics = vprofile.get_intrinsics()
            print(intrinsics)
            
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        filtered_colormap = cv2.applyColorMap(cv2.convertScaleAbs(filtered_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.circle(filtered_colormap, (200,200), 10, (255,0,255), 5)

        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('Unfiltered', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Unfiltered', images)
        cv2.namedWindow('Filtered', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Filtered', filtered_colormap)

        # Show images
        cv2.waitKey(5)
        # break

finally:

    # Stop streaming
    pipeline.stop()