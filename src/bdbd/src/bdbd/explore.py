'''
Control of motion, using simple collission detection to randomly explore
'''
import time
import os
import traceback
from enum import Enum
import rospy
from bdbd.msg import RoadBlocking
from bdbd.msg import MotorsRaw
from bdbd.msg import GamepadEvent
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion

try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty

def main():

    # constants
    MIN_RATE = 0.2 # how long to wait for a message until we panic and stop
    OBSTACLE_CLOSE_CENTER = 0.15 # how close is too close for obstacle, in meters
    OBSTACLE_CLOSE_SIDES = 0.25
    DROPOFF_CLOSE = 0.4 # how much viable road before we declare a dropoff
    HYST = 0.2 # fractional change required to change state
    SPEED = 0.7

    # enums
    class Moving(Enum):
        STOPPED = 1
        FORWARD = 2
        LEFT = 3 # moving to the left
        RIGHT = 4 # moving to the right
        REVERSE = 5
        ROTATE_LEFT = 6
        ROTATE_RIGHT = 7
        CORRECTING = 8
        NONE = 9 # Use this to force motor update after a change

    class Blocking(Enum):
        LEFT = 1
        CENTER = 2
        RIGHT = 3
        DOWN_LEFT = 4
        DOWN_CENTER = 5
        DOWN_RIGHT = 6
        COLLIDED = 7

    # functions
    def msg_cb(msg):
        # callback just queues messages for processing
        queue.put(msg)

    def updateBlocking(roadBlocking, blocking):
        # update blocking state from distances

        factor = (1. + HYST) if Blocking.LEFT in blocking else 1.0
        if roadBlocking.leftObstacleDepth < OBSTACLE_CLOSE_SIDES * factor:
            blocking.add(Blocking.LEFT)
        else:
            blocking.discard(Blocking.LEFT)

        factor = (1. + HYST) if Blocking.CENTER in blocking else 1.0
        if roadBlocking.centerObstacleDepth < OBSTACLE_CLOSE_CENTER * factor:
            blocking.add(Blocking.CENTER)
        else:
            blocking.discard(Blocking.CENTER)

        factor = (1. + HYST) if Blocking.RIGHT in blocking else 1.0
        if roadBlocking.rightObstacleDepth < OBSTACLE_CLOSE_SIDES * factor:
            blocking.add(Blocking.RIGHT)
        else:
            blocking.discard(Blocking.RIGHT)

        factor = (1. + HYST) if Blocking.DOWN_LEFT in blocking else 1.0
        if roadBlocking.leftRoadDepth < DROPOFF_CLOSE * factor:
            blocking.add(Blocking.DOWN_LEFT)
        else:
            blocking.discard(Blocking.DOWN_LEFT)

        factor = (1. + HYST) if Blocking.DOWN_CENTER in blocking else 1.0
        if roadBlocking.centerRoadDepth < DROPOFF_CLOSE * factor:
            blocking.add(Blocking.DOWN_CENTER)
        else:
            blocking.discard(Blocking.DOWN_CENTER)

        factor = (1. + HYST) if Blocking.DOWN_RIGHT in blocking else 1.0
        if roadBlocking.rightRoadDepth < DROPOFF_CLOSE * factor:
            blocking.add(Blocking.DOWN_RIGHT)
        else:
            blocking.discard(Blocking.DOWN_RIGHT)

    def getStateFromSaying(saying):
        if not 'robot' in saying: return
        if 'go forward' in saying:
            return Moving.FORWARD
        if 'stop' in saying:
            return Moving.STOPPED
        return None

    def getNewMovement(desired_state, actual_state, blocking):
        '''
        Determine a new direction and state
        '''
        if actual_state in (Moving.STOPPED, Moving.NONE):
            actual_state = desired_state

        if desired_state not in [Moving.FORWARD]:
            new_state = Moving.STOPPED
        else:
            if actual_state == Moving.FORWARD:
                if {Blocking.CENTER, Blocking.DOWN_CENTER} & blocking:
                    new_state = Moving.REVERSE
                elif {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking:
                    new_state = Moving.LEFT
                elif {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                    new_state = Moving.RIGHT
                else:
                    new_state = Moving.FORWARD

            elif actual_state == Moving.REVERSE:
                # have we finished our reverse?
                if not ({Blocking.DOWN_CENTER, Blocking.CENTER} & blocking):
                    # choose new direction to have best chance of success
                    if {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                        new_state = Moving.ROTATE_RIGHT
                    else:
                        new_state = Moving.ROTATE_LEFT
                else:
                    new_state = Moving.REVERSE

            elif actual_state == Moving.LEFT or actual_state == Moving.ROTATE_LEFT:
                if {Blocking.LEFT, Blocking.DOWN_LEFT, Blocking.CENTER, Blocking.DOWN_CENTER} & blocking:
                    new_state = Moving.REVERSE
                elif {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking:
                    new_state = actual_state
                else:
                    new_state = Moving.FORWARD
            
            elif actual_state == Moving.RIGHT or actual_state == Moving.ROTATE_RIGHT:
                if {Blocking.RIGHT, Blocking.DOWN_RIGHT, Blocking.CENTER, Blocking.DOWN_CENTER} & blocking:
                    new_state = Moving.REVERSE
                elif {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                    new_state = actual_state
                else:
                    new_state = Moving.FORWARD

            else:
                raise RuntimeError('Unexpected actual_state {}'.format(actual_state))

        return new_state

    def do_movement(moving_state):
        left, right = (0.0, 0.0)
        if moving_state == Moving.STOPPED:
            pass
        elif moving_state == Moving.FORWARD:
            left, right = (SPEED, SPEED)
        elif moving_state == Moving.REVERSE:
            left, right = (-.7 * SPEED, -.7 * SPEED)
        elif moving_state == Moving.LEFT:
            right = SPEED
            left = 0.4 * SPEED
        elif moving_state == Moving.RIGHT:
            left = SPEED
            right = 0.4 * SPEED
        elif moving_state == Moving.ROTATE_RIGHT:
            left = SPEED
        elif moving_state == Moving.ROTATE_LEFT:
            right = SPEED
        else:
            rospy.logwarn('unexpected moving state')
        return (left, right)

    # main code
    rospy.init_node('explore')
    queue = Queue()
    blocking_sub = rospy.Subscriber('/bdbd/detectBlocking/roadBlocking', RoadBlocking, msg_cb)
    gamepad_sub = rospy.Subscriber('/bdbd/gamepad/events', GamepadEvent, msg_cb)
    motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)

    blocking = set()
    desired_state = Moving.STOPPED
    actual_state = Moving.STOPPED
    left = 0.0
    right = 0.0

    rospy.loginfo('{} starting with PID {}'.format(__name__, os.getpid()))

    while not rospy.is_shutdown():
        try:
            msg = queue.get(True, MIN_RATE)
            msg_type = str(msg._type)

            if msg_type == 'bdbd/GamepadEvent':
                if msg.name == 'BTN_START' and msg.value == 1:
                    desired_state = Moving.FORWARD
                if msg.name == 'BTN_SELECT' and msg.value == 1:
                    desired_state = Moving.STOPPED

            elif msg_type == 'bdbd/RoadBlocking':
                updateBlocking(msg, blocking)
                new_state = getNewMovement(desired_state, actual_state, blocking)
                newleft, newright = do_movement(new_state)
                if newleft != left or newright != right:
                    rospy.loginfo('left is {:6.2f} right is {:6.2f}'.format(newleft, newright))
                    rospy.loginfo('blocking: {}'.format(blocking))
                left = newleft
                right = newright
                motor_pub.publish(left, right)

                if (actual_state != new_state):
                    rospy.loginfo('Robot changing state to ' + str(new_state))
                    actual_state = new_state
            else:
                rospy.logwarn("Unexpected message type {}".format(msg_type))

            # rospy.loginfo('left: {:6.2f} right: {:6.2f}'.format(left, right))

        except:
            exc = traceback.format_exc()
            rospy.logerr('Exception caught in movement:\n%s', exc)
            break
        
    left = 0.0
    right = 0.0
    motor_pub.publish(left, right)

if __name__ == '__main__':
    main()
