#!/usr/bin/env python
import math
import rospy
from bdbd.msg import GamepadEvent
from bdbd.msg import MotorsRaw
try:
    from Queue import Queue
except:
    from queue import Queue

PERIOD = 0.01 # How frequently to call, in seconds

class Do_Motors():
    def __init__(self):
        self.left = 0.0
        self.right = 0.0
        self.Z = 127 # raw signal from joystick (horizontal)
        self.RZ = 128 # raw signal from joystick (vertical)
        self.active = False
        self.event_queue = Queue()
        rospy.init_node('domotors')
        self.sub = rospy.Subscriber('gamepad/events', GamepadEvent, self.on_gamepad_event)
        self.pub = rospy.Publisher('motors/cmd_raw', MotorsRaw, queue_size=10)

    def on_gamepad_event(self, msg):
        self.event_queue.put(msg)

    def run(self):
        while not rospy.is_shutdown():
            while not self.event_queue.empty():
                msg = self.event_queue.get()
                rospy.loginfo('domotors message:' + str(msg))
                if msg.name == 'BTN_START' and msg.value == 1:
                    self.active = True
                    rospy.loginfo('Starting')
                    self.update_motors()

                elif msg.name == 'BTN_SELECT' and msg.value == 1:
                    self.active = False
                    rospy.loginfo('Stopping')
                    self.update_motors()

                elif msg.name == 'ABS_Z':
                    self.Z = msg.value
                    self.update_motors()

                elif msg.name == 'ABS_RZ':
                    self.RZ = msg.value
                    self.update_motors()

                else:
                    continue # ignore any other type of gamepad message

            rospy.sleep(PERIOD)

    def update_motors(self):
        if not self.active:
            self.left = 0.0
            self.right = 0.0
        else:
            x = (self.Z - 127) / 128. # ABS_Z zeroes at 127
            y = (128 - self.RZ) / 128. #ABS_RZ zeroes at 128
            norm = math.sqrt(x*x + y*y)
            rospy.loginfo('(x, y) is ({:6.3f}, {:6.3f})'.format(x, y))
            theta = math.atan2(y, x)
            gamma = theta - math.pi / 4.
            rospy.loginfo('theta: {:6.3f} gamma: {:6.3f} norm: {:6.3f}'.format(theta, gamma, norm))

            self.left = max(-1.0, min(1.0, math.sqrt(2.0) * norm * math.cos(gamma)))
            self.right = max(-1.0, min(1.0, math.sqrt(2.0) * norm * math.sin(gamma)))
            rospy.loginfo('(left, right) is ({:6.3f}, {:6.3f})'.format(self.left, self.right))

        self.pub.publish(self.left, self.right)

def main():
    runner = Do_Motors()
    try:
        runner.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()