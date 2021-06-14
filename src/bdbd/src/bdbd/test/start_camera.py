# use bdnodes to start the camera via topic

import rospy
import traceback
from bdbd_common.srv import NodeCommand
try:
    from Queue import Queue
except:
    from queue import Queue

service_queue = Queue()

def main():
    rospy.init_node('start_camera')
    svc = rospy.ServiceProxy('/bdnodes/topic', NodeCommand)
    print('waiting for service')
    rospy.wait_for_service('/bdnodes/topic')

    while not rospy.is_shutdown():
        try:
            rospy.loginfo('sending service request')
            result = svc('/bdbd/pantilt_camera/image_raw/compressed', 'start')
            print(result)

        except rospy.ServiceException:
            rospy.logwarn(traceback.format_exc())
        except:
            rospy.logerr('Exception while waiting for service, exiting')
            break
        break
    
main()