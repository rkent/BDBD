# scratchpad for various simple experiments

from bdbd_common.geometry import pose3to2, pose2to3, D_TO_R, transform2d
from bdbd_common.utils import fstr
from bdbd_common.pathPlan2 import PathPlan

startPose3 = pose2to3((0.0, 0.0, 0.0))
endPose3 = pose2to3((.4, .1, 30. * D_TO_R))

pp = PathPlan()
pp.start(startPose3, endPose3)
for seg in pp.path:
    print(fstr(seg,'8.5f'))

for seg in pp.path:
    # The control model operates in the robot frame, relative to the plan. So we need the deviation
    # of the actual location from the plan location, in the robot frame.
    near_wheel_m = transform2d(seg['end'], pp.frame_p, pp.frame_m)
    # near_wheel_m is the current origin of the wheel frame for the nearest point
    near_robot_m = transform2d(pp.robot_w, near_wheel_m, pp.frame_m)
    print(fstr(near_robot_m, '8.5f'))

#print(fstr(pose3to2(pose3), '10.7f'))
