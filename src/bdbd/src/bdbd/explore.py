'''
Control of motion, using simple collision detection to randomly explore
'''
import time
import os
import traceback
import random
import math
from enum import Enum
from copy import deepcopy
import rospy
import tf
from bdbd.msg import RoadBlocking
from bdbd.msg import MotorsRaw
from bdbd.msg import GamepadEvent
from bdbd.msg import AngledText
from bdbd.msg import SpeechAction
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from libpy.geometry import poseDistance, poseTheta, D_TO_R

try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty

# constants
MIN_RATE = 1.0 # how long to wait for a message until we panic and stop
OBSTACLE_CLOSE_CENTER = 0.20 # how close is too close for obstacle, in meters
OBSTACLE_CLOSE_SIDES = 0.30
DROPOFF_CLOSE = 0.30 # how much viable road before we declare a dropoff
HYST = 0.25 # fractional change required to change state
SPEED = 0.8
SPEED_MIN = 0.25
REVERSE_DISTANCE = 0.10 # How far to back up after forward obstacle
REVERSE_ANGLE = 55 # degrees to change direction after a reverse
DYNAMIC_DELAY = 0.15 # seconds to wait after motor change before recheck of status
MOTOR_RAMP_RATE = 0.2 # seconds for first-order motor speed ramp rate filter
MOTOR_RAMP_TOLERANCE = 0.05 # 

# proportional control parameters
THETA_GAIN = 3.0
DISTANCE_GAIN = 10.0
BACKING_LIMIT = .06
DISTANCE_TOLERANCE = .015

# enums
class Moving(Enum):
    STOPPED = 1
    FORWARD = 2
    LEFT = 3 # moving to the left
    RIGHT = 4 # moving to the right
    REVERSE = 5
    ROTATE_LEFT = 6
    ROTATE_RIGHT = 7
    TARGET = 8 # calculate left/right to move toward a target

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
    # EXPLORE = 5
    STOPPED = 6
    ROTATE_LEFT_CLEAR = 7 # rotate until clear
    ROTATE_RIGHT_CLEAR = 8
    ROTATE = 9 # choose best direction of rotation base on blocking or pose
    TARGET = 10 # move to the target pose
    TARGET_POINT = 11 # move to target pose point

# classes
class Objective():
    # where to go, both long-term and short-term
    def __init__(self, direction, poseTarget=None):
        self.direction = direction
        self.poseTarget = poseTarget

# functions

queue = Queue()
def msg_cb(msg):
    # callback just queues messages for processing
    queue.put(msg)

def zeroPose(frame):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    return pose

def motorRamp(left, right):
    # limit rate of change of motors
    if hasattr(motorRamp, 'lastTime'):   
        deltaTime = time.time() - motorRamp.lastTime
        deltaLeft = left - motorRamp.lastSpeeds[0]
        deltaRight = right - motorRamp.lastSpeeds[1]
        if abs(deltaLeft) > MOTOR_RAMP_TOLERANCE or abs(deltaRight) > MOTOR_RAMP_TOLERANCE:
            factor = math.exp(-deltaTime / MOTOR_RAMP_RATE)
            left -= deltaLeft * factor
            right -= deltaRight * factor
    motorRamp.lastSpeeds = left, right
    motorRamp.lastTime = time.time()
    return left, right

def getLR(tfl, target_pose):
    # get motor left, right for a target pose with proportional control
    vmax = SPEED
    # determine the coordinates of the target relative to the robot
    target_pose = tfl.transformPose('rear_wheels', target_pose)
    target_p = target_pose.pose.position
    theta = math.atan2((target_p.y), (target_p.x))
    distance = math.sqrt(target_p.y**2 + target_p.x**2)

    # if distance is short, allow backing. If not, spin to proper orientation
    if distance < BACKING_LIMIT:
        right_speed = min(vmax, max(-vmax, DISTANCE_GAIN * distance * (math.cos(theta) + math.sin(theta))))
        left_speed = min(vmax, max(-vmax, DISTANCE_GAIN * distance * (math.cos(theta) - math.sin(theta))))
    else:
        mx = min(vmax, DISTANCE_GAIN * distance)
        if theta > 0.0:
            right_speed = mx
            left_speed = max(-mx, mx - THETA_GAIN * theta)
        else:
            left_speed = mx
            right_speed = max(-mx, mx + THETA_GAIN * theta)

    # scale the motors according to SPEED_MIN if needed
    speed = math.sqrt(left_speed**2 + right_speed**2) # not really a vector, just scaling
    if speed > 0.0 and speed < SPEED_MIN:
        left_speed = left_speed * (SPEED_MIN/speed)
        right_speed = right_speed * (SPEED_MIN/speed)

    rospy.loginfo('LR distance: {:6.3f} theta: {:6.3f} target x,y: {:6.3f}, {:6.3f} left, right: {:6.3f}, {:6.3f}'
        .format(distance, theta, target_p.x, target_p.y, left_speed, right_speed))
    return (left_speed, right_speed,)
    #return (0.0, 0.0,)
   
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

def getNewMovement(objectives, blocking, current_pose, last_pose, tfl):
    # Determine a new direction and state
    new_state = None

    while new_state is None:
        direction = objectives and objectives[0].direction or Direction.STOPPED
        if direction != Direction.STOPPED:
            rospy.logdebug('objectives.direction: {} blocking: {}'.format(direction, blocking))
        target_pose = objectives and objectives[0].poseTarget or None
        # rospy.loginfo('current_pose {}'.format(current_pose))
        current_pose = tfl.transformPose('map', current_pose)
        last_pose = tfl.transformPose('map', last_pose)
        if target_pose is not None:
            # rospy.logdebug('target_pose: {}'.format(target_pose))
            target_pose = tfl.transformPose('map', target_pose)

        if direction == Direction.STOPPED:
            new_state = Moving.STOPPED

        elif direction == Direction.TARGET:
            objectives.pop(0)
            # We'll accomplish the target in two phases: first, an LR phase that moves
            # the rear wheel position to a final position. Second, a rotation to the final angle.
            #
            objectives.insert(0, Objective(Direction.ROTATE, target_pose))

            # Calculate the rear wheel position at target, and move to it
            rear_target = deepcopy(target_pose)
            rospy.loginfo('TARGET pose: {}'.format(rear_target))
            base_position = tfl.transformPose('rear_wheels', zeroPose('base_link')).pose.position
            delta = base_position.x # assumes y offset from base to rear is zero
            theta = poseTheta(target_pose, zeroPose('map'))
            rear_target.pose.position.x -= delta * math.cos(theta)
            rear_target.pose.position.y -= delta * math.sin(theta)
            objectives.insert(0, Objective(Direction.TARGET_POINT, rear_target))
            rospy.loginfo('TARGET inserted move to ({:6.3f}, {:6.3f}) theta {:6.3f} delta {:6.3f}'
                .format(rear_target.pose.position.x, rear_target.pose.position.y, theta, delta))

        elif direction in [Direction.FORWARD, Direction.TARGET_POINT]:
            if direction == Direction.TARGET_POINT:
                # check if we are done
                target_p = tfl.transformPose('rear_wheels', target_pose).pose.position
                distance = math.sqrt(target_p.y**2 + target_p.x**2)
                rospy.loginfo('getNewMovement distance: {:6.3f}'.format(distance))
                if distance < DISTANCE_TOLERANCE:
                    # we reached our point
                    objectives.pop(0)
                    continue

            # move forward if possible
            if not {Blocking.CENTER, Blocking.DOWN_CENTER} & blocking:
                # TODO: make this work better with LR
                if {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking:
                    new_state = Moving.LEFT
                elif {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking:
                    new_state = Moving.RIGHT
                else:
                    # not blocking
                    new_state = Moving.TARGET if direction == Direction.TARGET_POINT else Moving.FORWARD

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
            rospy.logdebug('distance: {}'.format(current_distance))
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
            # used to insert a rotate decision in the future
            objectives.pop(0)

            if not target_pose:
                # choose a new rotation direction based on blocking
                br = {Blocking.RIGHT, Blocking.DOWN_RIGHT} & blocking
                bl = {Blocking.LEFT, Blocking.DOWN_LEFT} & blocking
                if not bl:
                    new_direction = Direction.ROTATE_LEFT_CLEAR
                elif not br:
                    new_direction = Direction.ROTATE_RIGHT_CLEAR
                else:
                    new_direction = random.choice([Direction.ROTATE_LEFT_CLEAR, Direction.ROTATE_RIGHT_CLEAR])
                rotate_direction = Direction.ROTATE_LEFT if new_direction == Direction.ROTATE_LEFT_CLEAR else Direction.ROTATE_RIGHT
                objectives.insert(0, Objective(new_direction, None))

                # But first rotate a minimum amount
                theta = REVERSE_ANGLE if rotate_direction == Direction.ROTATE_LEFT else -REVERSE_ANGLE
                q_orig = tf.transformations.quaternion_from_euler(0, 0, 0)
                q_rot = tf.transformations.quaternion_from_euler(0, 0, theta * D_TO_R)
                q_new = tf.transformations.quaternion_multiply(q_rot, q_orig)
                o_new = Quaternion()
                o_new.x = q_new[0]
                o_new.y = q_new[1]
                o_new.z = q_new[2]
                o_new.w = q_new[3]
                pose_new = tfl.transfomPose('base_link', current_pose)
                pose_new.pose.orientation = o_new
                pose_new = tfl.transformPose('map', pose_new)
            else:
                # choose rotation based on target pose
                pose_new = target_pose
                theta = poseTheta(current_pose, pose_new)
                rotate_direction = Direction.ROTATE_LEFT if theta > 0.0 else Direction.ROTATE_RIGHT

            objectives.insert(0, Objective(rotate_direction, pose_new))
            new_state = Moving.ROTATE_LEFT if rotate_direction == Direction.ROTATE_LEFT else Moving.ROTATE_RIGHT
            rospy.loginfo('Inserted rotate to theta with theta {} dir {}'.format(theta, rotate_direction))

        elif direction == Direction.ROTATE_LEFT:
            theta= poseTheta(current_pose, target_pose)
            delta_theta = poseTheta(current_pose, last_pose)
            rospy.loginfo('theta: {} dtheta: {}'.format(theta, delta_theta))
            if theta - delta_theta < 0.0: # end early
                objectives.pop(0)
            else:
                new_state = Moving.ROTATE_LEFT

        elif direction == Direction.ROTATE_RIGHT:
            theta= poseTheta(current_pose, target_pose)
            delta_theta = poseTheta(current_pose, last_pose)
            rospy.loginfo('theta: {} dtheta: {}'.format(theta, delta_theta))
            if theta + delta_theta > 0.0: # end early
                objectives.pop(0)
            else:
                new_state = Moving.ROTATE_RIGHT

        else:
            raise RuntimeError('unknown direction {}'.format(direction))
    if new_state != Moving.STOPPED:
        rospy.logdebug('new_state: {}'.format(new_state))
    return new_state, target_pose

def do_movement(moving_state, tfl, target_pose):
    left, right = (0.0, 0.0)
    if moving_state == Moving.STOPPED:
        pass
    elif moving_state == Moving.FORWARD:
        left, right = (SPEED, SPEED)
    elif moving_state == Moving.REVERSE:
        left, right = (-.7 * SPEED, -.7 * SPEED)
    elif moving_state == Moving.LEFT:
        right = 1.2 * SPEED
        left = .4 * SPEED
    elif moving_state == Moving.RIGHT:
        left = 1.2 * SPEED
        right = .4 * SPEED
    elif moving_state == Moving.ROTATE_RIGHT:
        left = SPEED
        right = -SPEED
    elif moving_state == Moving.ROTATE_LEFT:
        right = SPEED
        left = -SPEED
    elif moving_state == Moving.TARGET:
        (left, right) = getLR(tfl, target_pose)
    else:
        rospy.logwarn('unexpected moving state')
    return motorRamp(left, right)

# main code
def main():
    rospy.init_node('explore')
    lastChange = time.time()
    tfl = tf.TransformListener()
    # let the listener listen some
    rospy.sleep(2.0)
    blocking_sub = rospy.Subscriber('/bdbd/detectBlocking/roadBlocking', RoadBlocking, msg_cb)
    gamepad_sub = rospy.Subscriber('/bdbd/gamepad/events', GamepadEvent, msg_cb)
    action_sub = rospy.Subscriber('speechResponse/action', SpeechAction, msg_cb)
    objective_sub = rospy.Subscriber('/bdbd/explore/poseTarget', PoseStamped, msg_cb)
    motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
    sayit_pub = rospy.Publisher('/bdbd/sayit/text', String, queue_size=10)
    objectives = []
    blocking = set()
    actual_state = Moving.STOPPED
    new_state = None
    left = 0.0
    right = 0.0
    current_pose = None
    target_pose = None
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
                    objectives = [Objective(Direction.FORWARD, None)]

                if msg.name == 'BTN_SELECT' and msg.value == 1:
                    rospy.loginfo('Stop exploring')
                    objectives = []

                if msg.name == 'BTN_NORTH' and msg.value == 1:
                    # test of Direction.TARGET
                    rospy.loginfo('Inserted test target objective')
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.orientation.w = 1.0
                    pose.pose.position.x = 0.2
                    pose.pose.position.y = 0.2
                    objectives = [Objective(Direction.TARGET, pose)]

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
                        rospy.logdebug('No last_pose')
                        continue
                    new_state, target_pose = getNewMovement(objectives, blocking, current_pose, last_pose, tfl)

                newleft, newright = do_movement(new_state, tfl, target_pose)
                #rospy.loginfo('left: {} right: {} state: {}'.format(newleft, newright, new_state))
                # log motion changes unless in TARGET
                if new_state != Moving.TARGET and (newleft != left or newright != right):
                    rospy.logdebug('left is {:6.2f} right is {:6.2f}'.format(newleft, newright))
                    rospy.loginfo('blocking: {}'.format(blocking))
                    lastChange = time.time()
                motor_pub.publish(newleft, newright)
                left = newleft
                right = newright

                if (actual_state != new_state):
                    rospy.loginfo('Robot changing state to ' + str(new_state))
                    actual_state = new_state

            elif msg_type == 'bdbd/SpeechAction':
                command = msg.command

                rospy.loginfo('explore heard: {}'.format(command))
                if command == 'explore':
                    sayit_pub.publish('OK, start exploring')
                    objectives = [Objective(Direction.FORWARD, None)]
                elif command == 'stop':
                    sayit_pub.publish('OK, stop exploring')
                    objectives = []
    
            elif msg_type == 'geometry_msgs/PoseStamped':
                # this is an an objective addition, to move to a target pose. It overrides a final FORWARD objective,
                # or another target pose, at the end of the objective queue.
                while len(objectives):
                    final_objective = objectives[-1]
                    if final_objective.direction in (Direction.TARGET, Direction.FORWARD,):
                        final_objective.pop(-1)
                    else:
                        break
                objectives.append(Objective(Direction.TARGET, msg))

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
