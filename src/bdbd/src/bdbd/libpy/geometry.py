# Common geometry methods

import tf
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, PointStamped

D_TO_R = math.pi / 180. # degrees to radians
TWOPI = 2. * math.pi

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

def zeroPose(frame):
    # a zero PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rospy.Time(0)
    pose.pose.orientation.w = 1.0
    return pose

def zeroPoint(frame):
    # a zero PointStamped
    point = PointStamped()
    point.header.frame_id = frame
    point.header.stamp = rospy.Time(0)
    return point

def rotationCenter(frame, vx, vy, omega):
    # center of rotation in frame. See RKJ notebook 2020-07-10
    r = vx / omega
    a = vy / omega
    #print(r, a)

    center = zeroPoint(frame)
    center.point.x = -a
    center.point.y = r
    return center

def shortestPath(rho, phi, x, y):
    '''
        shortest path from (0, 0) to a point at (x, y), with ending orientation phi from current
        orientation. Travel in either straight lines, or on circles of radius rho.

        See RKJ notebook circa 2020-08-16
    '''

    # motion circles
    # sc : start circle
    sc_ccw = [0.0, rho]
    sc_cw = [0.0, -rho]

    # fc: end_circle
    fc_ccw = [x - rho * math.sin(phi), y + rho * math.cos(phi)]
    fc_cw = [x + rho * math.sin(phi), y - rho * math.cos(phi)]

    A = [0., 0.] # starting point
    B = [x, y]   # ending point
    solutions = []
    # direction: 0 == ccw, 1 == cw
    for start_dir in (0, 1):
        for end_dir in (0, 1):
            C = sc_ccw if start_dir == 0 else sc_cw  # start motion circle center
            D = fc_ccw if end_dir == 0 else fc_cw    # end motion circle center
            a = D[0] - C[0]
            b = D[1] - C[1]
            theta = math.atan2(b, a)
            tsq = a**2 + b**2
            if start_dir != end_dir and tsq - 4. * rho **2 < 0.:
                #print('dir: {} {} invalid'.format(start_dir, end_dir))
                pass
            else:
                if start_dir == end_dir:
                    ssq = tsq
                    beta = theta if start_dir == 0  else -theta
                else:
                    ssq = tsq - 4. * rho**2
                    psi = math.acos(2. * rho / math.sqrt(tsq))
                    alpha = psi - theta if start_dir == 0 else psi + theta
                    beta = math.pi/ 2. - alpha
                s = math.sqrt(ssq)
                beta = beta % TWOPI

                E = [rho * math.sin(beta), 1. - rho * math.cos(beta)] # transition from start circle to line
                if start_dir == 1:
                    E[1] = -E[1]
                # F is transition from line to final circle
                if end_dir == 0:
                    F = [D[0] + rho * math.sin(beta), D[1] - rho * math.cos(beta)]
                else:
                    F = [D[0] - rho * math.sin(beta), D[1] + rho * math.cos(beta)]

                # RKJ notebook 2020-08-26
                if start_dir == 0 and end_dir == 0:
                    gamma = phi - beta
                elif start_dir == 0 and end_dir == 1:
                    gamma = beta - phi
                elif start_dir == 1 and end_dir == 0:
                    gamma = beta + phi
                else:
                    gamma = -beta - phi

                gamma = gamma % TWOPI
                length = s + rho * beta + rho * gamma
                solution = {'dir': (start_dir, end_dir), 'length': length, 'E': E, 'F': F, 'beta': beta, 'gamma': gamma}
                solutions.append(solution)
                #print(solution)

    # return the best solution
    solution = solutions[0]
    for i in range(1, len(solutions)):
        if solutions[i]['length'] < solution['length']:
            solution = solutions[i]
    return solution
