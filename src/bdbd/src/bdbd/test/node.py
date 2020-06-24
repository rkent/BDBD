# skeleton node for use in testing
import rospy
from std_msgs.msg import String

def msg_cb(msg):
    print(msg._full_text)

def main():
    rospy.init_node('test')
    rospy.Subscriber('/bdbd/dtest', String, msg_cb)
    rospy.spin()

main()

