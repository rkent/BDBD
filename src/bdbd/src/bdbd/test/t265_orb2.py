#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function
import traceback

"""
RKJ 2021-08-21

ORB: demo of calculating and displaying orb features, starting from base of t265_stereo

This is a modified example from the realsense repo. The original used a
pipeline, but failed when both the t265 and sr305 were installed. This modified
version relies on the sensor protocol rather than pipeline.

END RKJ

This example shows how to use T265 intrinsics and extrinsics in OpenCV to
asynchronously compute depth maps from T265 fisheye images on the host.

T265 is not a depth camera and the quality of passive-only depth options will
always be limited compared to (e.g.) the D4XX series cameras. However, T265 does
have two global shutter cameras in a stereo configuration, and in this example
we show how to set up OpenCV to undistort the images and compute stereo depth
from them.

Getting started with python3, OpenCV and T265 on Ubuntu 16.04:

First, set up the virtual enviroment:

$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings

Then, for every new terminal:

$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_stereo.py        # Run the example
"""

# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np
from math import tan, pi
import time

# camera selection stuff
ctx = rs.context()
devices = ctx.query_devices()
device = None
sensor = None
left_sp = None
right_sp = None
for d in devices:
    print('device', d, d.is_tm2(), d.get_info(rs.camera_info.serial_number))
    if d.is_tm2():
        print('Found t265')
        device = d
        break

if device:
    for s in device.sensors:
        print('sensor:', s)
        sensor = s
        print('serial number', s.get_info(rs.camera_info.serial_number))
        for p in s.profiles:
            print('profile', p)
            name = p.stream_name()
            print('profile.stream_name()', name)
            if name == 'Fisheye 1':
                print('Found Fisheye 1')
                left_sp = p.as_video_stream_profile()
            elif name == 'Fisheye 2':
                print('Found Fisheye 2')
                right_sp = p.as_video_stream_profile()

"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.

The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:

https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Set up a mutex to share data between threads 
from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None,
              "newleft_ts": 0.0,
              "new_right_ts": 0.0,
              "newleft_data": None,
              "newright_data": None
              }

"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback(frame):
    try:
        global frame_data
        if frame.profile.stream_name() == "Fisheye 1":
            type = 'left'
        else:
            type='right'
        ts = frame.timestamp
        rs_data = frame.as_video_frame().get_data()
        np_data = np.asanyarray(rs_data)
    except:
        print(traceback.format_exc())

    frame_mutex.acquire()
    try:
        if type == 'left':
            frame_data['newleft_data'] = np_data
            frame_data['newleft_ts'] = ts
        else:
            frame_data['newright_data'] = np_data
            frame_data['newright_ts'] = ts
        if frame_data['newleft_ts'] == frame_data['newright_ts']:
            frame_data['left'] = frame_data['newleft_data']
            frame_data['right'] = frame_data['newright_data']
            frame_data['timestamp_ms'] = ts
    finally:
        frame_mutex.release()

sensor.open([left_sp, right_sp])
sensor.start(callback=callback)
orb = cv2.ORB_create(
    nfeatures=1000,
    edgeThreshold = 15,
    patchSize=15,
    fastThreshold=10
    )

try:
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE = 't265_orb'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)

    intrinsics = {"left"  : left_sp.get_intrinsics(),
                  "right" : right_sp.get_intrinsics()}

    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get the relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(left_sp, right_sp)

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
    stereo_height_px = 300          # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)
    R_right = R

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # Construct the left and right projection matrices, the only difference is
    # that the right projection matrix should have a shift along the x axis of
    # baseline*focal_length
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0]*stereo_focal_px

    # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
    # since we will crop the disparity later
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                  [0, 1,       0, -stereo_cy],
                  [0, 0,       0, stereo_focal_px],
                  [0, 0, -1/T[0], 0]])

    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}

    mode = "stack"
    print('Enter s for stack, o for overlay, q to quit')
    while True:
        start = time.time()
        # Check if the camera has acquired any frames
        frame_mutex.acquire()
        valid = frame_data["timestamp_ms"] is not None
        frame_mutex.release()

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            frame_mutex.acquire()
            frame_copy = {"left"  : frame_data["left"].copy(),
                          "right" : frame_data["right"].copy()}
            frame_mutex.release()

            # Undistort and crop the center of the frames
            center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                          map1 = undistort_rectify["left"][0],
                                          map2 = undistort_rectify["left"][1],
                                          interpolation = cv2.INTER_LINEAR),
                                  "right" : cv2.remap(src = frame_copy["right"],
                                          map1 = undistort_rectify["right"][0],
                                          map2 = undistort_rectify["right"][1],
                                          interpolation = cv2.INTER_LINEAR)}

            # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
            #disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

            # re-crop just the valid part of the disparity
            #disparity = disparity[:,max_disp:]

            # convert disparity to 0-255 and color it
            #disp_vis = 255*(disparity - min_disp)/ num_disp
            #disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
            #left_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
            #right_image = cv2.cvtColor(center_undistorted["right"][:,max_disp:], cv2.COLOR_GRAY2RGB)
            left_image = cv2.cvtColor(center_undistorted["left"], cv2.COLOR_GRAY2RGB)
            right_image = cv2.cvtColor(center_undistorted["right"], cv2.COLOR_GRAY2RGB)

            orb_image = left_image.copy()
            kp = orb.detect(orb_image, None)
            #print(dir(kp[0]))
            print('found {} keypoints'.format(len(kp)))

            # remove overlaps
            kp_mask = len(kp) * [True]
            for i in range(len(kp)):
                if not kp_mask[i]:
                    continue
                for j in range(i+1, len(kp)):
                    if not kp_mask[j]:
                        continue
                    overlap = kp[i].overlap(kp[i], kp[j])
                    #print(overlap)
                    if overlap > 0.10:
                        # reject if angles are close
                        angle_diff = 180 - abs(abs(kp[j].angle - kp[i].angle) - 180)
                        #print('angles {} {} diff {}'.format(kp[i].angle, kp[j].angle, angle_diff))
                        if angle_diff < 45:
                            # choose strongest kp
                            if kp[i].response > kp[j].response:
                                kp_mask[j] = False
                            else:
                                kp_mask[i] = False
                                break
            kpf = [kp[i] for i in range(len(kp)) if kp_mask[i]]
            print('len kpf', len(kpf))
            '''
            for i in range(len(kp)):
                print(' ')
                kp0=kp[i]
                for m in dir(kp0):
                    if m.startswith('__'):
                        continue
                    print(m, getattr(kp0, m))
            #kp, des = orb.compute(orb_image, kp)
            '''
            orb_image2 = cv2.drawKeypoints(orb_image, kpf, None, color=(0,255,0), flags=0)

            cv2.imshow(WINDOW_TITLE, np.hstack((left_image, orb_image2)))
        print('Processing time', time.time() - start)
        key = cv2.waitKey(200)
        if key == ord('s'): mode = "stack"
        if key == ord('o'): mode = "overlay"
        if key == ord('q'):
            break
finally:
    sensor.stop()
    sensor.close()
    print(traceback.format_exc())
