# respond to heard speech
import rospy

import os
import traceback
import enum
import time
from bdbd.msg import AngledText
from bdbd.msg import SpeechAction
from bdbd.srv import SpeechCommand
from std_msgs.msg import String
from std_msgs.msg import Bool

class TextStatus(enum.Enum):
    listen = 0
    understand = 1
    respond = 2
    speak = 3
    action = 4
    error = 5

class status():
    ''' object to hold global status'''
    last_sayit = ''

def main():
    def on_mike_status(msg):
        text_cb(None, msg.data)

    def text_cb(msg, mike_status=None):
        if not msg and mike_status is not None:
            status.mike = mike_status

        if msg:
            # process statement text
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
                elif words[1] == 'repeat':
                    action = 'repeat'
                else:
                    action = 'sayit'
                    detail = "I can't " + ' '.join(words[1:])
            else:
                action = 'chat'
                detail = statement

            rospy.loginfo('speechResponse requested action <{}>'.format(action))
            if action == 'chat':
                status.text = TextStatus.respond
                pixelring_pub.publish('think')
                result = chat_srv('chatservice', detail)
                if result and result.response:
                    pixelring_pub.publish('purple')
                    status.text = TextStatus.speak
                    status.last_sayit = result.response
                    sayit_response = sayit_srv('say', result.response)
                status.text = TextStatus.listen

            elif action == 'repeat':
                pixelring_pub.publish('purple')
                status.text = TextStatus.speak
                sayit_response = sayit_srv('say', status.last_sayit)
                status.text = TextStatus.listen
                
            elif action == 'explore' or action == 'stop':
                pixelring_pub.publish('purple')
                status.text = TextStatus.speak
                sayit_response = sayit_srv('say', 'OK ' + action)
                status.text = TextStatus.action
                action_pub.publish(action, detail)
                # flash led blue
                pixelring_pub.publish('blue')
                time.sleep(1)
                status.text = TextStatus.listen
            elif action == 'sayit':
                status.text = TextStatus.error
                rospy.logwarn('Unknown action')
                pixelring_pub.publish('red')
                sayit_response = sayit_srv('say', detail)
                status.text = TextStatus.listen

        # set pixelring led
        if status.text == TextStatus.listen:
            if status.mike:
                pixelring_pub.publish('listen')
            else:
                pixelring_pub.publish('spin')

    status.text = TextStatus.listen
    status.mike = False

    rospy.init_node('speechResponse')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    rospy.Subscriber('hearit/angled_text', AngledText, text_cb)
    rospy.Subscriber('mike/status', Bool, on_mike_status)
    action_pub = rospy.Publisher('speechResponse/action', SpeechAction, queue_size=10)
    chat_srv = rospy.ServiceProxy('chat', SpeechCommand)
    sayit_srv = rospy.ServiceProxy('sayit', SpeechCommand)
    pixelring_pub = rospy.Publisher('pixelring', String, queue_size=10)
    rospy.sleep(1)
    pixelring_pub.publish('spin')
    rospy.spin()

if __name__ == '__main__':
    main()