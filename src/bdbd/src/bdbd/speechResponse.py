# respond to heard speech
import rospy

import os
import traceback
from bdbd.msg import AngledText
from bdbd.msg import SpeechAction


def main():

    def text_cb(msg):
        statement = msg.text.lower()
        if not statement:
            return
        rospy.loginfo('processing text <{}>'.format(statement))
        words = statement.split()

        # robot commands
        action = None
        detail = None
        if words[0] == 'robot':
            if words[1].startswith('explore'):
                action = 'explore'
            elif words[1] == 'stop':
                action = 'stop'
        else:
            action = 'chat'
            detail = statement
        if action is not None:
            action_pub.publish(action, detail)
            rospy.loginfo('speechResponse requested action <{}>'.format(action))

    rospy.init_node('speechResponse')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    rospy.Subscriber('hearit/angled_text', AngledText, text_cb)
    action_pub = rospy.Publisher('speechResponse/action', SpeechAction, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()