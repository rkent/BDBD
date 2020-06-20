'''
Examination of RNN results
'''
import tensorflow
import tensorflow.keras as keras
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Odometry
import tf
import math
from functools import partial
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import random
import time

D_TO_R = 3.1415926535 / 180. # degrees to radians
SEC_PER_STEP = 0.025
MAX_SPEED = 1.0
EPS = 1.0e-7
NITERS = 100

class LR():
    '''
    function for left, right as a combination of splines
    '''
    def __init__(self, lefts, rights, taus, vmax):
        assert len(lefts) == len(rights), 'len(lefts) = {} len(rights) = {}'.format(len(lefts), len(rights))
        assert len(taus) == len(lefts) -1, 'len(taus) = {}, len(lefts) = {}'.format(len(taus), len(lefts))
        self.cs_left = None
        self.cs_right = None
        self.vmax = vmax
        x = 0
        xes = [0]
        for tau in taus:
            x += tau
            xes.append(x)
        #print('xes: {}'.format(xes))
        #print('lefts: {}'.format(lefts))
        #print('rights: {}'.format(rights))
        self.cs_left = CubicSpline(xes, lefts, bc_type = 'clamped')
        self.cs_right = CubicSpline(xes, rights, bc_type = 'clamped')

    def speeds(self, x):

        return np.transpose(np.array([np.clip(self.cs_left(x), -self.vmax, self.vmax), 
                np.clip(self.cs_right(x), -self.vmax, self.vmax)]))

# TODO: move these to libpy
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

def check_theta(i, xs, ys, thetas, target=None):
    # Have we reached the target theta?
    assert target is not None
    if i < 15:
        return False
    last_error = abs(target - thetas[-2])
    current_error = abs(target - thetas[-1])
    if current_error > last_error:
        return True
    else:
        return False

def check_distance(i, xs, ys, thetas, target=None):
    assert target is not None
    if i < 15:
        return False
    xt = target[0]
    yt = target[1]
    last_distance_sq = (xs[-2] - xt)**2 + (ys[-2] - yt)**2
    current_distance_sq = (xs[-1] - xt)**2 + (ys[-1] - yt)**2
    if current_distance_sq > last_distance_sq:
        return True
    else:
        return False

def do_predict(np_inputs):
    results = []

    class Result():
        def __init__(self):
            self.xs = []
            self.ys = []
            self.thetas = []
            self.vxs = []
            self.vys = []
            self.thetadots = []

    pred = model.predict(np_inputs)
    print('pred.shape: {}'.format(pred.shape))

    for k in range(pred.shape[0]):
        x = 0.0
        y = 0.0
        theta = 0.0

        result = Result()
        for pred_i in range(pred.shape[1]):
            vx = pred[k, pred_i, 0]
            vy = pred[k, pred_i, 1]
            thetadot = pred[k, pred_i, 2]
            result.vxs.append(vx)
            result.vys.append(vy)
            result.thetadots.append(thetadot)
            theta += SEC_PER_STEP * thetadot
            result.thetas.append(theta)
            sintheta = math.sin(theta)
            costheta = math.cos(theta)
            x += SEC_PER_STEP * (vx * costheta - vy * sintheta)
            y += SEC_PER_STEP * (vx * sintheta + vy * costheta)
            result.xs.append(x)
            result.ys.append(y)
            
            speedx = float(np_inputs[k, pred_i, 0])
            speedy = float(np_inputs[k, pred_i, 1])
            #print('i: {} speedx: {:5.2f} speedy: {:5.2f} theta (degrees): {:6.2f} x: {:6.3f} y: {:6.3f} vx: {:6.3f} vy: {:6.3f}'.format(
            #    pred_i, speedx, speedy, theta / D_TO_R, x, y, vx, vy))
            
            #print('i:{} speedx:{:5.2f} speedy:{:5.2f} theta(degrees):{:6.2f} x:{:6.3f} vx:{:6.3f} dt:{:6.3f}'.format(
            #    pred_i, speedx, speedy, theta / D_TO_R, x, vx, thetadot))
            
        results.append(result)

    return results

def loss_calc(result):
    #np_ins = np.array(speeds, dtype=float, ndmin=3)

    thetas = result.thetas
    xs = result.xs
    ys = result.ys
    vxs = result.vxs
    vys = result.vys
    thetadots = result.thetadots

    start = time.time()

    # Now we will calculate the loss metrics
    lf = [400, 100, 100, 100, 10, .02, 1000]
    
    # 1) the square of the final distance error
    pos_loss = (xs[-1] - delta_x)**2 + (ys[-1] - delta_y)**2

    # 2) the square of the final theta error, in radians squared
    theta_loss = (thetas[-1] - final_theta)**2

    # 3) the square of the final x velocity error, in m**2/s**2
    #print('vxs[-1]: {} target_twist.twist.linear.x: {}'.format(vxs[-1], target_twist.twist.linear.x))
    xvel_loss = (vxs[-1] - target_twist.twist.linear.x)**2

    # 4) the square of the final y velocity error
    yvel_loss = (vys[-1] - target_twist.twist.linear.y)**2

    # 5) the square of the final angular velocity error
    thetadot_loss = (thetadots[-1] - target_twist.twist.angular.z)**2

    # 6) the total time
    time_loss = len(xs)

    # 7) average jerk.
    jerksum = 0.0
    for i in range(1, len(vxs) - 1):
        jerksum += (vxs[i-1] - 2.*vxs[i] + vxs[i+1])**2 + (vys[i-1] - 2.*vys[i] + vys[i+1])**2
    jerksum_loss = jerksum / sum(taus)

    raw_losses = (pos_loss, theta_loss, xvel_loss, yvel_loss, thetadot_loss, time_loss, jerksum_loss)
    #print('raw: pos_loss {:8.5f} theta_loss {:8.5f} xvel_loss {:8.5f} yvel_loss {:8.5f} thetadot_loss {:8.5f} time_loss {:8.5f} jerksum_loss {:8.5f}'
    #    .format(pos_loss, theta_loss, xvel_loss, yvel_loss, thetadot_loss, time_loss, jerksum_loss))

    scaled_losses = []
    for i in range(len(raw_losses)):
        scaled_losses.append(raw_losses[i] * lf[i])

    total_loss = sum(scaled_losses)
    #print('total: {:8.5f} pos_loss {:8.5f} theta_loss {:8.5f} xvel_loss {:8.5f} yvel_loss {:8.5f} thetadot_loss {:8.5f} time_loss {:8.5f} jerksum_loss {:8.5f}'
    #    .format(total_loss, *scaled_losses))

    #print('x: {:7.3f} y: {:7.3f} theta: {:7.3f} vx: {:7.3f} vy: {:7.3f} dtheta: {:7.3f}'.format(xs[-1], ys[-1], thetas[-1], vxs[-1], vys[-1], thetadots[-1]))

    return total_loss, scaled_losses

def make_speeds(lefts, rights, taus, tees):
    lr = LR(lefts, rights, taus, MAX_SPEED)

    np_speeds = lr.speeds(tees)
    return(np_speeds)

model = keras.models.load_model('data/modelRnnFortieth15a')
print(model.summary())

# main program constants

# testing
target_odometry = Odometry()
current_pose = PoseStamped()
current_pose.header.frame_id = 't265_pose_frame'
current_pose.pose.orientation.w = 1.0
target_pose = PoseStamped()
target_pose.pose = target_odometry.pose.pose
target_twist = TwistStamped()
target_twist.twist = target_odometry.twist.twist
target_pose.header.frame_id = 't265_pose_frame'
target_twist.header.frame_id = 't265_pose_frame'
target_twist.header.frame_id = 't265_pose_frame'
target_pose.pose.orientation.w = 1.0

# set some sample values

target_pose.pose.position.x = 0.0
target_pose.pose.position.y = 0.0

qa_orig = q_to_array(current_pose.pose.orientation)
qa_rot = tf.transformations.quaternion_from_euler(0, 0, 0 * D_TO_R)
qa_new = tf.transformations.quaternion_multiply(qa_rot, qa_orig)
q_new = array_to_q(qa_new)
target_pose.pose.orientation = q_new

# determine theta to new direction
delta_y = target_pose.pose.position.y - current_pose.pose.position.y
delta_x = target_pose.pose.position.x - current_pose.pose.position.x
delta_theta = math.atan2(delta_y, delta_x)
final_theta = poseTheta(current_pose, target_pose)

# test speeds
lefts =  [0.0, 1.0, 1.0, 0.0, 0.0]
rights = [0.0, -1.0, -1.0, 0.0, 0.0]

# now nsteps is just the starting values
nsteps = 25
taus = [nsteps, nsteps, nsteps, nsteps]
ltees = [0]
itime = 0
for j in range(len(taus)):
    for k in range(int(taus[j])):
        itime += 1
        ltees.append(itime)
tees = np.array(ltees)
speeds = make_speeds(lefts, rights, taus, tees)
#print(speeds)
for count in range(10):
    # Just kidding, we'll really set them here
    for i in range(len(speeds)):
        speeds[i, 0] = random.random()
        speeds[i, 1] = random.random()
    np_ins = np.array(speeds, ndmin=3)
    start = time.time()
    pred = do_predict(np_ins)
    print('pred time: {}'.format(time.time() - start))
fig = plt.figure(figsize=(6,4))
plt.plot(tees, pred[0].vxs)
plt.plot(tees, pred[0].xs)
plt.plot(tees, pred[0].thetas)
#plt.plot(tees, speeds[:, 1])
#plt.plot(tees, speeds[:, 0])
plt.pause(.02)
input('?')
exit()

if True:
    start = time.time()
    vars = lefts + rights + taus
    print('taus: {}'.format(taus))
    dvars = (nallvars+1)*[0]
    losses = (nallvars+1)*[0]
    dlosses = (nallvars+1)*[0]
    newvars = (nallvars+1)*[0]
    dl_dvs = (nallvars+1)*[0]

    speedses = []


    for i in range(nallvars+1):
        cvars = list(vars)
        if i > 0:
            cvars[i-1] += dvars[i]
        ltees = [0]
        itime = 0
        ctaus = cvars[nspeedvars:]
        #print('ctaus: {}'.format(ctaus))
        for j in range(len(ctaus)):
            for k in range(int(ctaus[j])):
                itime += 1
                ltees.append(itime)
        tees = np.array(ltees)
        #print('len(tees): {}'.format(len(tees)))
        speeds = make_speeds(cvars[0:len(lefts)], cvars[len(lefts):nspeedvars], cvars[nspeedvars:], tees)
        #print('speeds.shape: {}'.format(speeds.shape))
        speedses.append(speeds)

        # plot
        if i == 0:
            fig.clf()
            plt.plot(tees, speeds[:, 0])
            plt.plot(tees, speeds[:, 1])
            plt.pause(.02)
            #input('?')

    # some of the speeds may be lengthened for derivative calculations
    speeds_length = 0
    for speeds in speedses:
        if len(speeds) > speeds_length:
            speeds_length = len(speeds)
    # pad the speeds arrays to the same length when adding to the np array
    np_ins = np.zeros((nallvars+1, speeds_length, 2))
    for i in range(len(speedses)):
        speeds = speedses[i]
        if len(speeds) == speeds_length:
            np_ins[i, :, 0] = speeds[:, 0]
            np_ins[i, :, 1] = speeds[:, 1]
        else:
            pad = np.zeros((speeds_length - len(speeds),))
            np_ins[i, :, 0] = np.concatenate((pad, speeds[:, 0]))
            np_ins[i, :, 1] = np.concatenate((pad, speeds[:, 1]))

    print('np_ins.shape from speedses: {}'.format(np_ins.shape))
    pred_results = do_predict(np_ins)
    total_losses = []
    for pred_result in pred_results:
        total_loss, scaled_losses = loss_calc(pred_result)
        total_losses.append(total_loss)
        if len(total_losses) == 1:
            r = pred_result
            # This is the real loss, before pertubations to calc derivates
            print('scaled_losses: {}'.format(scaled_losses))
            print('x: {:6.3f} y: {:6.3f} theta: {:6.3f} vx: {:6.3f} vy: {:6.3f} thetadot: {:6.3f}'.format(r.xs[-1], r.ys[-1], r.thetas[-1],
                r.vxs[-1], r.vys[-1], r.thetadots[-1] ))

    for i in range(nallvars+1):
        losses[i] = total_losses[i]
        if i == 0:
            print('\ntotal_loss: {:8.5f} alpha: {}'.format(total_losses[i], alpha))
            #input("Press Enter to continue")

    for i in range(1, nallvars+1):
        dlosses[i] = losses[i] - losses[0]
        dl_dv = dlosses[i] / (dvars[i] + EPS)
        dl_dvs[i] = dl_dv
        if i < nspeedvars+1:
            newvars[i] = -gamma * last_changes[i-1] + vars[i-1] - alpha * dl_dv
        else:
            deltaV = -gamma * last_changes[i-1] + 1000. * alpha * dl_dv
            '''
            if deltaV > 0.:
                deltaV = max(1.01, deltaV)
            else:
                deltaV = min(-1.01, deltaV)
            '''
            newvars[i] = max(5.0, vars[i-1] - deltaV)
    #print('last_loss {} losses[0] {}'.format(last_loss, losses[0]))
    print('dl_dvs: {}'.format(dl_dvs))
    print('vars: {}'.format(vars))
    print('newvars: {}'.format(newvars))
    if last_loss and prev_vars and last_loss < losses[0]:
    #if False:
        alpha /= 2.
        lefts = prev_vars[0:len(lefts)]
        rights = prev_vars[len(lefts):nspeedvars]
        taus = prev_vars[nspeedvars:]
    else:
        last_loss = losses[0]
        lefts = newvars[1:len(lefts)+1]
        rights = newvars[len(lefts)+1:nspeedvars+1]
        for i in range(nspeedvars+1, nallvars+1):
            taus[i - nspeedvars - 1] = int(newvars[i] + .5)
        alpha *= 1.05
        if prev_vars:
            for kk in range(1, len(newvars)):
                last_changes[kk - 1] = newvars[kk] - prev_vars[kk - 1]
        prev_vars = lefts + rights + taus
    delta_time = time.time() - start
    print('Time per iteration: {}'.format(delta_time))

plt.show()
#exit()
