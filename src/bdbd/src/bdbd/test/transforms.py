# Use ros-compatible python, like python2 on ros melodic

import tf
import rospy
import geometry_msgs.msg

rospy.init_node('test_transforms')
tl = tf.TransformListener()
rospy.sleep(1.0)

transforms = tl.getFrameStrings()

base_frame = 'base_link'
new_frame = 'pantilt_camera'
'''
for ts in transforms:
    if ts == base_frame:
        continue
    print(ts)
    try:
        print(tl.lookupTransform(base_frame, ts, rospy.Time(0)))
    except:
        print('error')
'''

p1 = geometry_msgs.msg.PoseStamped()
p1.header.frame_id = "base_link"
p1.pose.orientation.w = 1.0    # Neutral orientation
p_in_new = tl.transformPose(new_frame, p1)
print("transform\n{}\n from {} to {} is\n{}".format(p1.pose, base_frame, new_frame, p_in_new.pose))

