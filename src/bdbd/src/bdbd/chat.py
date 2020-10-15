# chat with user
import rospy

import os
import traceback
from std_msgs.msg import String
from bdbd.msg import SpeechAction
from bdbd.srv import SpeechCommand
from bdbd.libpy.cleverbot import Cleverbot
from bdbd_common.srv import Dialog

class Chatbot():
    def __init__(self):
        self.cleverbot = Cleverbot()
        self.bdbdDialog = rospy.ServiceProxy('/bdbd/dialog', Dialog)

    def __call__(self, statement):
        try:
            useDialog = False
            try:
                rospy.wait_for_service('/bdbd/dialog', .05)
                useDialog = True
            except:
                pass
            if useDialog:
                saying = self.bdbdDialog(statement, "").response
                rospy.loginfo('bdbd/dialog response is {}'.format(saying))
            else:
                response = self.cleverbot.getReply(statement)
                rospy.loginfo('chatBot response is {}'.format(response))
                saying = "Sorry, brain freeze"
                if response['status_code'] == 200:
                    saying = response['output'].encode('ascii', 'ignore')
                else:
                    rospy.logwarn('cleverbot response code {}'.format(response['status_code']))
            return saying
        except:
            rospy.logerr('Error from chatbot: {}'.format(traceback.format_exc()))
            return "Sorry, error"
            
def main():

    def action_cb(msg):
        command = msg.command.lower()
        if command not in ['chat', 'chatservice']:
            return
        statement = msg.detail
        rospy.loginfo('chatbot will reply to <{}>'.format(statement))
        try:
            saying = chatbot(statement)
            if command == 'chat':
                sayit_pub.publish(saying)
            else:
                return saying
        except:
            rospy.logerr('Error from action_cb: {}'.format(traceback.format_exc()))
        

    rospy.init_node('chat')
    chatbot = Chatbot()
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    rospy.Subscriber('speechResponse/action', SpeechAction, action_cb)
    rospy.Service('chat', SpeechCommand, action_cb)
    sayit_pub = rospy.Publisher('sayit/text', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()