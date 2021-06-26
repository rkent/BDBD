# Ensure nodes are launched (when including DoerRequest in code is impractical)
import rospy
import sys
from bdbd_common.doerRequest import DoerRequest

def main(argv):
    rospy.init_node('ensureDoer', anonymous=True)
    topic_doers = argv[0]
    service_doers = argv[1]
    topics = topic_doers.split()
    services = service_doers.split()
    for topic in topics:
        rospy.loginfo('ensure_doer for topic topic {}'.format(topic))
    for service in services:
        rospy.loginfo('ensure_doer for service {}'.format(service))
    topic_doers = ['/t265/odom/sample']
    service_doers = []
    for topic in topics:
        try:
            rospy.loginfo('Ensuring doer for topic {}'.format(topic))
            DoerRequest().ensure_doer(topic, 'topic', timeout=30.0)
            rospy.loginfo('OK doer for {}'.format(topic))
        except:
            rospy.logwarn('topic {} failed to start'.format(topic))
    for service in services:
        try:
            rospy.loginfo('Ensuring doer for service {}'.format(service))
            DoerRequest().ensure_doer(service, 'service', timeout=30.0)
            rospy.loginfo('OK doer for {}'.format(service))
        except:
            rospy.logwarn('service {} failed to start'.format(service))
    rospy.spin()

if __name__ == '__main__':
    main('aaa')
    #main(sys.argv[1:])