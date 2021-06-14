# respond to heard speech
import rospy

import os
import sys
import traceback
import enum
import time
from bdbd.msg import AngledText
from bdbd.msg import SpeechAction
from bdbd.srv import SpeechCommand
from bdbd_common.srv import NodeCommand
from std_msgs.msg import String
from std_msgs.msg import Bool
from libpy.Battery import Battery
from geometry_msgs.msg import PoseStamped
from libpy.geometry import poseTheta, D_TO_R
import tf
import json

# Load the behaviors file to get allowable behaviors
Behaviors_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'behaviors.json')
with open(Behaviors_path) as behaviors_file:
    Behaviors = json.load(behaviors_file)

# action defaults to word. Word only has to start with.
commands = [
    {   'word': 'move'
    },
    {   'word': 'stop'
    },
    {   'word': 'repeat'
    },
    {   'word': 'report',
        'options': ['battery', 'behavior', 'position']
    },
    {   'word': 'begin',
        'options': Behaviors.keys()
    },
    {   'word': 'end',
        'options': Behaviors.keys()
    },
    {   'word': 'quiet'
    },
    {   'word': 'talk'
    }
]

def helpcommands():
    result = "I only understand"
    for command in commands:
        result += ' ' + command['word']
    return result

def helpoptions(command):
    result = " sorry, after " + command['word'] + " I only understand"
    for option in command['options']:
        result += " " + option
    return result

# TODO these do very little but sometimes affect pixel ring
class TextStatus(enum.Enum):
    listen = 0
    understand = 1
    respond = 2
    speak = 3
    action = 4
    error = 5

class status():
    ''' object to hold global status'''
    last_sayit = 'I only repeat what I say when I am chatting'
    talk = True
    sayonce = False

def main():
    def on_mike_status(msg):
        text_cb(None, msg.data)

    def text_cb(msg, mike_status=None):
        if not msg and mike_status is not None:
            status.mike = mike_status

        if msg:
            # process statement text
            sayit = None
            statement = msg.text.lower()
            if not statement:
                return
            rospy.loginfo('processing text <{}>'.format(statement))
            words = statement.split()
            while len(words) < 4:
                words.append('')

            action = None
            detail = None

            if words[0] == 'robot':
                for command in commands:
                    if words[1].startswith(command['word']):
                        action = command['word']
                        detail = '_MISSING_'
                        if 'options' in command:
                            for option in command['options']:
                                if words[2].startswith(option):
                                    detail = option

                            if detail == '_MISSING_':
                                action = 'sayit'
                                rospy.loginfo('did not recognize option {}'.format(words[2]))
                                sayit = helpoptions(command)
                                break
                        break
            elif status.talk:
                action = 'chat'
                detail = statement

            # robot commands
            '''
            if words[0] == 'robot':
                if words[1].startswith('explore'):
                    action = 'explore'
                elif words[1] == 'stop':
                    action = 'stop'
                elif words[1] == 'repeat':
                    action = 'repeat'
                elif words[1] == 'report':
                    action = 'report'
                    option = None
                    if words[2] == 'battery':
                        option = 'battery'
                    elif words[2].startswith('behavior'):
                        option = 'behavior'
                    elif words[2] == 'position':
                        option = 'position'
                elif words[1].startswith('behav'):
                    action = 'behavior'
                    behavior = None
                    option = None

                    if words[2] in Behaviors:
                        behavior = words[2]
                    # possible variants
                    elif words[2].startswith('explor'):
                        behavior = 'explore'
                    elif words[2].startswith('object'):
                        behavior = 'objects'

                    if words[3] == 'start':
                        option = 'start'
                    elif words[3] == 'stop':
                        option = 'stop'

                    if behavior and option:
                        detail = behavior + ' ' + option
                    else:
                        action = 'sayit'
                        detail = "I don't know how to " + ' '.join(words[2:])
                elif words[1] == 'status':
                    action = 'sayit'
                    detail = 'OK status {}'.format(words[2])
                    if words[2] == 'quiet':
                        status.talk = False
                    elif words[2] == 'talk':
                        status.talk = True
                    else:
                        detail = "I don't understand {}".format(words[2])
                else:
                    action = 'sayit'
                    detail = "I can't " + ' '.join(words[1:])
            else:
                action = 'chat'
                detail = statement

            '''

            rospy.loginfo('speechResponse requested action <{}>'.format(action))
            if action is None:
                sayit = helpcommands()

            if action == 'move' or action == 'stop':
                pixelring_pub.publish('purple')
                status.text = TextStatus.action
                action_pub.publish(action, detail)
                sayit = 'OK  ' + action + ' request sent'
                # flash led blue
                #pixelring_pub.publish('blue')

            elif action == 'repeat':
                sayit = status.last_sayit

            elif action == 'report':
                if detail == 'battery':
                    sayit = 'battery voltage is ' + str(battery())
                elif detail == 'behavior':
                    behaviors_str = bdnodes_srv('', 'report').response
                    rospy.loginfo('reported active behaviors: {}'.format(behaviors_str))
                    sayit = 'active behaviors are ' + behaviors_str
                elif detail == 'position':
                    base_pose = PoseStamped()
                    base_pose.header.frame_id = 'base_link'
                    base_pose.pose.orientation.w = 1.0
                    zero_pose = PoseStamped()
                    zero_pose.header.frame_id = 'map'
                    zero_pose.pose.orientation.w = 1.0
                    try:
                        map_pose = tfl.transformPose('map', base_pose)
                        x = map_pose.pose.position.x
                        y = map_pose.pose.position.y
                        theta = poseTheta(zero_pose, map_pose) / D_TO_R
                        sayit = 'x is {:6.2f} meters, y is {:6.2f} meters, and theta is {:6.0f} degrees.'.format(x, y, theta)
                    except tf.LookupException:
                        sayit = 'Transforms inactive' 
                else:
                    sayit = "I can't report on " + detail

            elif action == 'begin':
                pixelring_pub.publish('purple')
                status.text = TextStatus.action
                sayit = 'OK starting ' + detail
                bdnodes_srv(detail, 'start')

            elif action == 'end':
                pixelring_pub.publish('purple')
                status.text = TextStatus.action
                sayit = 'OK stopping ' + detail
                bdnodes_srv(detail, 'stop')

            elif action == 'quiet':
                status.talk = False

            elif action == 'talk':
                status.talk = True
                sayit = "OK I can talk now"

            elif action == 'chat':
                status.text = TextStatus.respond
                pixelring_pub.publish('think')
                try:
                    sayit = chat_srv('chatservice', detail).response
                    status.last_sayit = sayit
                except rospy.ServiceException:
                    rospy.logwarn('ROS service error: {}'.format(sys.exc_info()[1]))
                    sayit = "not feeling chatty today. I'll be quiet until you say robot talk"
                    status.talk = False
                    status.sayonce = True

            elif action == 'sayit':
                pass

            elif action is not None:
                status.text = TextStatus.error
                rospy.logwarn('Unknown action: {}'.format(action))
                pixelring_pub.publish('red')
                time.sleep(1)
                sayit = "sorry I can't " + ' '.join(words[1:])

            if sayit:
                if status.talk or status.sayonce:
                    pixelring_pub.publish('purple')
                    status.text = TextStatus.speak
                    status.sayonce = False
                    try:
                        sayit_srv('say', sayit)
                    except rospy.ServiceException:
                        rospy.logwarn('ROS service error: {}'.format(sys.exc_info()[1]))
                else:
                    rospy.loginfo('Wanted to say: {}'.format(sayit))
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
    tfl = tf.TransformListener()

    # Load the behaviors file to get allowable behaviors
    Behaviors_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'behaviors.json')
    rospy.loginfo('Loading behaviors file')
    with open(Behaviors_path) as behaviors_file:
        Behaviors = json.load(behaviors_file)

    rospy.Subscriber('hearit/angled_text', AngledText, text_cb)
    rospy.Subscriber('mike/status', Bool, on_mike_status)
    action_pub = rospy.Publisher('speechResponse/action', SpeechAction, queue_size=10)
    chat_srv = rospy.ServiceProxy('chat', SpeechCommand)
    sayit_srv = rospy.ServiceProxy('sayit', SpeechCommand)
    bdnodes_srv = rospy.ServiceProxy('/bdnodes/behavior', NodeCommand)
    pixelring_pub = rospy.Publisher('pixelring', String, queue_size=10)
    battery = Battery()
    rospy.sleep(1)
    pixelring_pub.publish('spin')
    rospy.spin()

if __name__ == '__main__':
    main()