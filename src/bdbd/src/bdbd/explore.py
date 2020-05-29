'''
Control of motion, using simple collision detection to randomly explore
'''
import time
import os
import traceback
import random
import math
from enum import Enum
import rospy
import tf
from bdbd.msg import RoadBlocking
from bdbd.msg import MotorsRaw
from bdbd.msg import GamepadEvent
from geometry_msgs.msg import PoseStamped, Quaternion

try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty

# constants
MIN_RATE = 0.2 # how long to wait for a message until we panic and stop
OBSTACLE_CLOSE_CENTER = 0.15 # how close is too close for obstacle, in meters
OBSTACLE_CLOSE_SIDES = 0.30
DROPOFF_CLOSE = 0.2 # how much viable road before we declare a dropoff
HYST = 0.25 # fractional change required to change state
SPEED = 0.8
REVERSE_DISTANCE = 0.10 # How far to back up after forward obstacle
REVERSE_ANGLE = 55 # degrees to change direction after a reverse
D_TO_R = 3.1415926535 / 180. # degrees to radians
DYNAMIC_DELAY = 0.15 # seconds to wait after motor change before recheck of status

# enums
class Moving(Enum):
    STOPPED = 1
    FORWARD = 2
    LEFT = 3 # moving to the left
    RIGHT = 4 # moving to the right
    REVERSE = 5
    ROTATE_LEFT = 6
    ROTATE_RIGHT = 7

class Blocking(Enum):
    LEFT = 1
    CENTER = 2
    RIGHT = 3
    DOWN_LEFT = 4
    DOWN_CENTER = 5
    DOWN_RIGHT = 6

class Direction(Enum):
    FORWARD = 1
    REVERSE = 2
    ROTATE_LEFT = 3
    ROTATE_RIGHT = 4
    EXPLORE = 5
    STOPPED = 6
    ROTATE_LEFT_CLEAR = 7 # rotate until clear
    ROTATE_RIGHT_CLEAR = 8
    ROTATE = 9 # choose best direction of rotation

# classes
class Objective():
    # where to go, both long-term and short-term
    def __init__(self, direction, poseTarget=None):
        self.direction = direction
        self.poseTarget = poseTarget

# functions

def poseDistance(pose1, pose2):
    # calculate the distance between two PoseStamped types in the same frame
    if pose1.header.frame_id != pose2.header.frame_id:
        raise RuntimeError('poses must be in the same frame')
    p1 = pose1.pose.position
    p2 = pose2.pose.position
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def poseTheta(pose1, pose2):
    # calculate the rotation on z axis between two PoseStamped types
    if pose1.header.frame_id != pose2.header.frame_id:
        raise RuntimeError('poses must be in the same frame')
    q1 = q_to_array(pose1.pose.orientation)
    q2 = q_to_array(pose2.pose.orientation)
    a1 = tf.transformations.euler_from_quaternion(q1)
    a2 = tf.transformations.euler_from_quaternion(q2)
    return (a2[2] - a1[2])

def q_to_array(orientation):
    # return 4-element array from geometry_msgs/Quaternion.msg
    return [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]

def array_to_q(array):
    #return geometry_msgs/Quaternion.msg from 4-element list
    q = Quaternion()
    q.x = array[0]
    q.y = array[1]
    q.z = array[2]
    q.w = array[3]
    return q

queue = Queue()
def msg_cb(msg):
    # callback just queues messages for processing
    queue.put(msg)

def updateBlocking(roadBlocking, blocking):
    rospy.loginfo('roadBlocking: obstacles {:5.3f} {:5.3f} {:5.3f} road: {:5.3f} {:5.3f} {:5.3f}'.format(
        roadBlocking.leftObstacleDepth, roadBlocking.centerObstacleDepth, roadBlocking.rightObstacleDepth,
        roadBlocking.leftRoadDepth, roadBlocking.centerRoadDepth, roadBlocking.rightRoadDepth
    ))
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

def getDirectionFromSaying(saying):
    if 'robot' in saying:
        if 'go forward' in saying:
            return Direction.FORWARD
        if 'stop' in saying:
            return Direction.STOPPED
    return None

def getNewMovement(objectives, blocking, current_pose, last_pose, tfl):
    # Determine a new direction and state
    new_state = None

    while new_state is None:
        direction = objectives and objectives[0].direction or Direction.STOPPED
        if direction != Direction.STOPPED:
            directions = []
            for objective in objectives:
                directions.append(objective.direction) 
            rospy.loginfo('objectives.direction: {}'.format(directions))
            rospy.loginfo('getNewMovement direction: {} blocking: {}'.format(direction, blocking))
        target_pose = objectives and objectives[0].poseTarget or None
        # rospy.loginfo('current_pose: {}'.format(current_pose))
        current_pose = tfl.transformPose('map', current_pose)
        last_pose = tfl.transformPose('map', last_pose)
        if target_pose is not None:
            # rospy.loginfo('target_pose: {}'.format(target_pose))
            target_pose = tfl.transformPose('map', target_pose)

        if direction == Direction.STOPPED:
            new_state = Moving.STOPPED

        elif direction in [Direction.FORWARD, Direction.EXPLORE]:
            # move forward if possible
            if not {Blocking.CENTER, Blocking.DOWN_CENTER} & blocking:
                if {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking:
                    new_state = Moving.LEFT
                elif {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                    new_state = Moving.RIGHT
                else:
                    new_state = Moving.FORWARD
            else:
                # reverse then rotate
                objectives.insert(0, Objective(Direction.ROTATE, None))
                new_pose = tfl.transformPose('base_link', current_pose)
                new_pose.pose.position.x -= REVERSE_DISTANCE
                new_pose = tfl.transformPose('map', new_pose)
                objectives.insert(0, Objective(Direction.REVERSE, new_pose))
                new_state = Moving.REVERSE

        elif direction == Direction.REVERSE:
            # reverse until we are moving away from target pose
            current_distance = poseDistance(current_pose, target_pose)
            last_distance = poseDistance(last_pose, target_pose)
            rospy.loginfo('distance: {}'.format(current_distance))
            if current_distance < last_distance:
                # we are getting closer, keep reversing
                new_state = Moving.REVERSE
            else:
                # we are done with our reverse (or collided)
                objectives.pop(0)
                continue

        elif direction == Direction.ROTATE_LEFT_CLEAR:
            if {Blocking.CENTER, Blocking.DOWN_CENTER, Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                new_state = Moving.ROTATE_LEFT
            else:
                objectives.pop(0)

        elif direction == Direction.ROTATE_RIGHT_CLEAR:
            if {Blocking.CENTER, Blocking.DOWN_CENTER, Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking:
                new_state = Moving.ROTATE_RIGHT
            else:
                objectives.pop(0)

        elif direction == Direction.ROTATE:
            # choose a new rotation direction based on current position
            br = {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking
            bl = {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking
            bc = {Blocking.CENTER, Blocking.DOWN_CENTER} & blocking
            if not bl:
                new_direction = Direction.ROTATE_LEFT_CLEAR
            elif not br:
                new_direction = Direction.ROTATE_RIGHT_CLEAR
            else:
                new_direction = random.choice([Direction.ROTATE_LEFT_CLEAR, Direction.ROTATE_RIGHT_CLEAR])

            objectives.pop(0)
            objectives.insert(0, Objective(new_direction, None))

        elif direction == Direction.ROTATE_LEFT:
            theta = poseTheta(current_pose, target_pose)
            delta_theta = poseTheta(current_pose, last_pose)
            rospy.loginfo('theta: {} dtheta: {}'.format(theta, delta_theta))
            if theta < 0.0 or delta_theta > 0.0:
                objectives.pop(0)
            else:
                new_state = Moving.ROTATE_LEFT

        elif direction == Direction.ROTATE_RIGHT:
            theta = poseTheta(current_pose, target_pose)
            delta_theta = poseTheta(current_pose, last_pose)
            rospy.loginfo('theta: {} dtheta: {}'.format(theta, delta_theta))
            if theta > 0.0 or delta_theta < 0.0:
                objectives.pop(0)
            else:
                new_state = Moving.ROTATE_RIGHT

        else:
            raise RuntimeError('unknown direction {}'.format(direction))
    if new_state != Moving.STOPPED:
        rospy.loginfo('new_state: {}'.format(new_state))
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
        left = .5 * SPEED
    elif moving_state == Moving.RIGHT:
        left = SPEED
        right = .5 * SPEED
    elif moving_state == Moving.ROTATE_RIGHT:
        left = SPEED
        right = -SPEED
    elif moving_state == Moving.ROTATE_LEFT:
        right = SPEED
        left = -SPEED
    else:
        rospy.logwarn('unexpected moving state')
    return (left, right)

# main code
def main():
    rospy.init_node('explore')
    lastChange = time.time()
    tfl = tf.TransformListener()
    # let the listener listen some
    rospy.sleep(2.0)
    blocking_sub = rospy.Subscriber('/bdbd/detectBlocking/roadBlocking', RoadBlocking, msg_cb)
    gamepad_sub = rospy.Subscriber('/bdbd/gamepad/events', GamepadEvent, msg_cb)
    motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
    objectives = []
    blocking = set()
    actual_state = Moving.STOPPED
    new_state = None
    left = 0.0
    right = 0.0
    current_pose = None
    last_pose = None

    rospy.loginfo('{} starting with PID {}'.format(__name__, os.getpid()))

    while not rospy.is_shutdown():
        try:
            try:
                msg = queue.get(True, MIN_RATE)
            except Empty:
                rospy.logwarn('explore message queue timeout')
                continue
            msg_type = str(msg._type)

            if msg_type == 'bdbd/GamepadEvent':
                if msg.name == 'BTN_START' and msg.value == 1:
                    # clear objective stack to start afresh
                    rospy.loginfo('Start exploring')
                    objectives = [Objective(Direction.EXPLORE, None)]

                if msg.name == 'BTN_SELECT' and msg.value == 1:
                    rospy.loginfo('Stop exploring')
                    objectives = []

            elif msg_type == 'bdbd/RoadBlocking':
                if not tfl.canTransform('base_link', 'map', rospy.Time()):
                    rospy.logwarn('No transform from map to base_link')
                    new_state = Moving.STOPPED
                else:
                    updateBlocking(msg, blocking)
                    last_pose = current_pose
                    current_pose = PoseStamped()
                    current_pose.header.frame_id = 'base_link'
                    current_pose.pose.orientation.w = 1.0
                    current_pose = tfl.transformPose('map', current_pose)
                
                    if time.time() < lastChange + DYNAMIC_DELAY:
                        #rospy.loginfo('Give change time')
                        continue
                    if last_pose is None:
                        rospy.loginfo('No last_pose')
                        continue
                    new_state = getNewMovement(objectives, blocking, current_pose, last_pose, tfl)

                newleft, newright = do_movement(new_state)
                if newleft != left or newright != right:
                    rospy.loginfo('left is {:6.2f} right is {:6.2f}'.format(newleft, newright))
                    rospy.loginfo('blocking: {}'.format(blocking))
                    lastChange = time.time()
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
