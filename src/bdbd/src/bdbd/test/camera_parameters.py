import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
#pipeline = rs.pipeline()
#config = rs.config()
context = rs.context()
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
devices = context.devices
for i in range(devices.size()):
    device = devices[i]
    name = device.get_info(rs.camera_info.name)
    print('found device: {}'.format(name))
    if 'SR305' in name:
        sr305 = device
    if 'T265' in name:
        t265 = device

print(sr305.query_sensors())
print(t265.query_sensors())
depth = sr305.first_depth_sensor()
print(depth)
print(depth.get_supported_options())
color = sr305.first_color_sensor()
print(color)
print(color.get_supported_options())
#print(color.get_option(rs.option.enable_auto_exposure))
#color.set_option(rs.option.enable_auto_exposure, True)
print(color.get_option(rs.option.enable_auto_exposure))
print(depth.get_option(rs.option.depth_units))

pose = t265.first_pose_sensor()
print(pose.get_supported_options())
#print(t265.first_fisheye_sensor())
#print(t265.first_motion_sensor())
