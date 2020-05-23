#!/usr/bin/env python
import rospy
from std_msgs.msg import String
try:
    from Queue import Queue
except:
    from queue import Queue
from espeakng import ESpeakNG

PERIOD = 0.1 # update rate in seconds

class SayIt():
    def __init__(self):
        self._espeak = ESpeakNG(voice='en-gb-x-gbclan')
        self._espeak.volume = 40
        rospy.init_node('sayit')
        self._sub = rospy.Subscriber('sayit/text', String, self.on_sayit_text)
        self._queue = Queue()

    def on_sayit_text(self, msg):
        text = msg.data
        rospy.loginfo('Request to say: ' + text)
        self._queue.put(text)

    def run(self):
        while not rospy.is_shutdown():
            while not self._queue.empty():
                # Don't allow requests to stack up
                while self._queue.qsize() > 2:
                    self._queue.get()
                text = self._queue.get()
                rospy.loginfo('Saying:' + text)
                self._espeak.say(text, sync=True)

            rospy.sleep(PERIOD)

def main():
    sayit = SayIt()
    sayit._queue.put('This is BDBD robot')
    sayit.run()

if __name__ == '__main__':
    main()