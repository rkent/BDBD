# locate people and chase them.
import rospy
try:
    from Queue import Queue
except:
    from queue import Queue

import os
import traceback
import time
from bdbd_common.msg import PanTilt

def main():
    rospy.init_node('peoplechase')
    pantilt_pub = rospy.Publisher('/bdbd/pantilt', PanTilt, queue_size=10)

    # main loop
    time.sleep(1) # initial pub does not work without this delay
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    pantilt_pub.publish(90, 20)
    rospy.spin()

if __name__ == '__main__':
    main()