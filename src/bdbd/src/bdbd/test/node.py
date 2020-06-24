# skeleton node for use in testing
import rospy
import os
from std_msgs.msg import String

def main():
    class C():
        def __init__(self):
            self.v = 'me'
        def cb(self, msg):
            print('i am me')
            print(self.v)
            filename = '/home/kent/github/rkent/bdbd/src/bdbd/src/bdbd/behaviors.json'
            print(os.stat(filename).st_mtime)
    c = C()
    rospy.init_node('test')
    rospy.Subscriber('/bdbd/dtest', String, c.cb)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
main()
