# sends a message at a fixed rate for timing tests
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('msgrate')

pub = rospy.Publisher('/msgrate', Odometry, queue_size=1)
rate = rospy.Rate(100)
while not rospy.is_shutdown():
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    pub.publish(odom)
    rate.sleep()
