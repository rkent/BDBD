# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs

context = rs.context()
devices = context.devices
for i in range(devices.size()):
    device = devices[i]
    name = device.get_info(rs.camera_info.name)
    print('found device: {}'.format(name))
    if 'SR305' in name:
        sr305 = device
    if 'T265' in name:
        t265 = device
#sr305.hardware_reset()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 10)
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 10)

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height
print('width: {}, height: {}'.format(w, h))

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 4)

while True:
    # Grab camera data
    try:
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        depth_frame = decimate.process(depth_frame)
        vf = depth_frame.as_video_frame()
        #print('vf width: {} vf height: {}'.format(vf.width, vf.height))
        points = pc.calculate(depth_frame)
        parr = np.asarray(points.get_vertices())
        #darr = np.array(depth_frame.data)
        rp = np.resize(parr, (vf.height, vf.width))
        print('\n')
        h13 = int(vf.height/3)
        w13 = int(vf.width/3)
        h23 = 2*h13
        w23 = 2*w13
        print(rp[h13, w13], rp[h13, w23])
        print(rp[h23, w13], rp[h23, w23])
        #print('parr: {} darr: {}'.format(parr.shape, darr.shape))
        #print([p for p in parr[3]])
        #print([d for d in darr[0:4]])
        #print(rp[0, 3])
        #break
    except KeyboardInterrupt:
        break

# Stop streaming
pipeline.stop()
