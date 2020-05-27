# test relocation of axes of a particular quarternion, to confirm transformation
# from base frame to depth and pointcloud

import rospy
import tf
import geometry_msgs.msg 
t = tf.TransformerROS(True, rospy.Duration(10.0))
m = geometry_msgs.msg.TransformStamped()
m.header.frame_id = 'base'
m.child_frame_id = 'depth'
m.transform.rotation.x = -.5
m.transform.rotation.y = .5
m.transform.rotation.z = -.5
m.transform.rotation.w = .5
t.setTransform(m)
print(t.lookupTransform('base', 'depth', rospy.Time(0)))

px = geometry_msgs.msg.PointStamped()
px.header.frame_id = 'base'
px.point.x = 1.0

pxx = t.transformPoint('depth', px)
print('1,0,0', pxx)

py = geometry_msgs.msg.PointStamped()
py.header.frame_id = 'base'
py.point.y = 1.0

pyy = t.transformPoint('depth', py)
print('0,1,0', pyy)

pz = geometry_msgs.msg.PointStamped()
pz.header.frame_id = 'base'
pz.point.z = 1.0

pzz = t.transformPoint('depth', pz)
print('0,0,1', pzz)

