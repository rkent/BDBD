# Use ros-compatible python, like python2 on ros melodic

import tf
import rospy
import geometry_msgs.msg

rospy.init_node('test_transforms')
tl = tf.TransformListener()
rospy.sleep(1.0)

transforms = tl.getFrameStrings()

base_frame = 't265_pose_frame'
for ts in transforms:
    if ts == base_frame:
        continue
    print(ts)
    try:
        print(tl.lookupTransform(base_frame, ts, rospy.Time(0)))
    except:
        print('error')

p1 = geometry_msgs.msg.PoseStamped()
p1.header.frame_id = "base_link"
p1.pose.orientation.w = 1.0    # Neutral orientation
p_in_base = tl.transformPose("/floor", p1)
print("Position of the floor in the robot base:", p_in_base)

