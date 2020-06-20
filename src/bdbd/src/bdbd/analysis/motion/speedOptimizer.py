'''
Functions used to provide optimal speed sequences
'''
import tensorflow
import tensorflow.keras as keras
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import cumtrapz
import math
import matplotlib.pyplot as plt

class LR():
    ''' class for left, right as a combination of splines

        left_edges, right_edges: ([float]) desired values of left, right speeds at points in time
        taus: ([float]) time interval (seconds) for each of the left, right intervals
        vmax: (float): a max value of velocity, used for value clipping
    '''
    def __init__(self, left_edges, right_edges, taus, vmax, bc_type='natural', rho=0.5, lin_factor=0.85):
        assert len(left_edges) == len(right_edges), 'len(lefts) = {} len(rights) = {}'.format(len(lefts), len(rights))
        assert len(taus) == len(left_edges) - 1, 'len(taus) = {}, len(lefts) = {}'.format(len(taus), len(lefts))
        self.cs_left = None # cubic spline function to generate lefts
        self.cs_right = None # cubic spline function to generate rights
        self.vmax = vmax
        self.rho = rho
        self.lin_limit = lin_factor * vmax
        # generate corresponding time values matching lefts, rights
        t = 0.0
        tees = [0.0]
        for tau in taus:
            t += tau
            tees.append(t)
        #print('rights: {}'.format(rights))
        self.cs_left = CubicSpline(tees, left_edges, bc_type=bc_type)
        self.cs_right = CubicSpline(tees, right_edges, bc_type=bc_type)

    def softclip(self, v):
        r = v
        if v > self.lin_limit:
            r = self.lin_limit + (self.vmax - self.lin_limit) * (1.0 - math.exp(-self.rho*(v - self.lin_limit)/(self.vmax - self.lin_limit)))
        elif v < -self.lin_limit:
            r = -self.lin_limit -(self.vmax - self.lin_limit) * (1.0 - math.exp(-self.rho*(-v - self.lin_limit)/(self.vmax - self.lin_limit)))
        return r

    def speeds(self, tees):
        lefts = []
        rights = []
        for t in tees:
            lefts.append(self.softclip(self.cs_left(t)))
            rights.append(self.softclip(self.cs_right(t)))
        return (lefts, rights)

    def f_left(self, t):
        return self.softclip(self.cs_left(t))

    def f_right(self, t):
        return self.softclip(self.cs_right(t))

def make_tees(taus, deltat):
    # generated t values with deltat separation, given a list of intervals.
    tees = []
    t = 0.0
    tmax = sum(taus)
    while t < tmax:
        tees.append(t)
        t += deltat
    tees.append(tmax)
    return tees

def print_list(ll, ss):
    print('{}: '.format(ss), end='')
    for item in ll:
        print('{:6.3f} '.format(item), end='')
    print('')

'''
Given multiple input series, return predicted values at each point in time.
Python lists are converted into appropriate form for tensorflow model. Different
length input lists are extended to the same length.

    model: keras model
    f_inputses: [ left(), right() ]: function to create the inputs
    xmax: total time for input
    deltat: chane in t for each interval in prediction sequence
'''
def predict(model, f_inputses, tmax, deltat):

    # create the inputses
    speedses = []
    for f_inputs in f_inputses:
        t = 0.0
        speeds = []
        f_left = f_inputs[0]
        f_right = f_inputs[1]
        while t < tmax + deltat:
            t += deltat
            speeds.append((f_left(t), f_right(t),))
        speedses.append(speeds)
    np_speedses = np.array(speedses, ndmin=3)
    #print('tmax: {} len(speedses): {}'.format(tmax, len(speedses)))
    return model.predict(np_speedses)

def integrate_rates(predictions, deltat):
    # integrate predictions of vx, yy, thetadot to get x, y, theta
    class Result():
        def __init__(self):
            self.xs = None
            self.ys = None
            self.thetas = None
            self.vxs = None
            self.vys = None
            self.thetadots = None
        def __str__(self):
            return 'xs: {:7.5f} ys: {:7.5f} thetas: {:7.5f} vxs: {:7.5f} vys: {:7.5f} thetadots: {:7.5f}'.format(
                self.xs[-1], self.ys[-1], self.thetas[-1], self.vxs[-1], self.vys[-1], self.thetadots[-1])

    ps = predictions
    n = ps.shape[1]
    tees = np.linspace(0.0, (n -1) * deltat, num=n)
    results = []

    for k in range(ps.shape[0]):
        result = Result()
        result.vxs = ps[k, :, 0]
        result.vys = ps[k, :, 1]
        result.thetadots = ps[k, :, 2]
        result.thetas = cumtrapz(result.thetadots, tees, initial=0.0)
        sinthetas = np.sin(result.thetas)
        costhetas = np.cos(result.thetas)
        map_vxs = result.vxs * costhetas - result.vys * costhetas
        map_vys = result.vxs * sinthetas + result.vys * sinthetas
        result.xs = cumtrapz(map_vxs, tees, initial=0.0)
        result.ys = cumtrapz(map_vys, tees, initial=0.0)
        results.append(result)

    return results

def calc_losses(result, tees, lastt, targets):
    lasti = len(tees) - 1
    for i in range(len(tees)):
        if tees[i] > lastt:
            lasti = i
            break

    def interpolate(a, tees, lasti, lastt):
        return ( a[lasti-1] + (a[lasti] - a[lasti-1])*(lastt - tees[lasti-1]) / (tees[lasti] - tees[lasti-1]) )

    #np_ins = np.array(speeds, dtype=float, ndmin=3)

    thetas = result.thetas
    xs = result.xs
    ys = result.ys
    vxs = result.vxs
    vys = result.vys
    thetadots = result.thetadots

    # Now we will calculate the loss metrics
    lf = [
        1000,  # 1) the square of the final distance error
        200,  # 2) the square of the final theta error, in radians squared
        200,  # the square of the final x velocity error
        0,  # 4) backing penalty
        10,   # 5) the square of the final angular velocity error
        0.00, # 6) the total time
        200.0  # 7) total jerk
    ]
    '''
    lf = [
        10,  # 1) the square of the final distance error
        200,  # 2) the square of the final theta error, in radians squared
        20,  # the square of the final x velocity error
        0,  # 4) backing penalty
        10,   # 5) the square of the final angular velocity error
        0.00, # 6) the total time
        200.0  # 7) total jerk
    ]
    '''

    
    xlast = interpolate(xs, tees, lasti, lastt)
    ylast = interpolate(ys, tees, lasti, lastt)
    thetalast = interpolate(thetas, tees, lasti, lastt)
    vxlast = interpolate(vxs, tees, lasti, lastt)
    vylast = interpolate(vys, tees, lasti, lastt)
    thetadotlast = interpolate(thetadots, tees, lasti, lastt)
    v_lasts = [xlast, ylast, thetalast, vxlast, vylast, thetadotlast]

    # 1) the square of the final distance error
    pos_loss = (xlast - targets.x)**2 + (ylast - targets.y)**2

    # 2) the square of the final theta error, in radians squared
    theta_loss = (thetalast - targets.theta)**2

    # 3) the square of the final x velocity error, in m**2/s**2
    #print('vxs[-1]: {} target_twist.twist.linear.x: {}'.format(vxs[-1], target_twist.twist.linear.x))
    xvel_loss = (vxlast - targets.vx)**2

    # 4) penalize backing
    backing_loss = 0.0
    for vs in vxs:
        if vs < 0.:
            backing_loss += vs * vs

    # 5) the square of the final angular velocity error
    thetadot_loss = (thetadotlast - targets.thetadot)**2

    # 6) the total time
    time_loss = lastt

    # 7) total jerk.
    jerksum = 0.0
    for i in range(1, lasti):
        newjerk = (vxs[i-1] - 2.*vxs[i] + vxs[i+1])**2 + (vys[i-1] - 2.*vys[i] + vys[i+1])**2
        jerksum += newjerk
    jerksum_loss = jerksum

    raw_losses = (pos_loss, theta_loss, xvel_loss, backing_loss, thetadot_loss, time_loss, jerksum_loss)
    #print('raw: pos_loss {:8.5f} theta_loss {:8.5f} xvel_loss {:8.5f} yvel_loss {:8.5f} thetadot_loss {:8.5f} time_loss {:8.5f} jerksum_loss {:8.5f}'
    #    .format(pos_loss, theta_loss, xvel_loss, yvel_loss, thetadot_loss, time_loss, jerksum_loss))

    scaled_losses = []
    for i in range(len(raw_losses)):
        scaled_losses.append(raw_losses[i] * lf[i])

    total_loss = sum(scaled_losses)
    #print('total: {:8.5f} pos_loss {:8.5f} theta_loss {:8.5f} xvel_loss {:8.5f} yvel_loss {:8.5f} thetadot_loss {:8.5f} time_loss {:8.5f} jerksum_loss {:8.5f}'
    #    .format(total_loss, *scaled_losses))

    #print('x: {:7.3f} y: {:7.3f} theta: {:7.3f} vx: {:7.3f} vy: {:7.3f} dtheta: {:7.3f}'.format(xs[-1], ys[-1], thetas[-1], vxs[-1], vys[-1], thetadots[-1]))

    return total_loss, scaled_losses, v_lasts

def vary_vars(left_edges, right_edges, taus):
    # returns slightly varied versions of variables to use in derivate calculations. The first is
    # the base, unvaried.
    vars = left_edges + right_edges + taus
    varses = [vars]
    last_speed_i = len(left_edges) + len(right_edges)
    for i in range(len(vars)):
        cvarses = list(vars)
        if i < last_speed_i:
            cvarses[i] += .001 #change in speed
        else:
            cvarses[i] += .001 # change in time
        varses.append(cvarses)
    return varses

def get_dldv(model, left_edges_base, right_edges_base, taus_base, targets, deltat):
    # get the losses, and derivatives of the loss function for each variable
    varses = vary_vars(left_edges_base, right_edges_base, taus_base)
    lasti_left = len(left_edges_base)
    lasti_right = lasti_left + len(right_edges_base)
    f_inputses = []
    tmax = 0.0
    for vars in varses:
        left_edges = vars[0:lasti_left]
        right_edges = vars[lasti_left:lasti_right]
        taus = vars[lasti_right:]
        lr = LR(left_edges, right_edges, taus, vmax=1.0)
        f_inputses.append((lr.f_left, lr.f_right,))
        if tmax < sum(taus):
            tmax = sum(taus)
    ps = predict(model, f_inputses, tmax=tmax, deltat=deltat)
    results = integrate_rates(ps, deltat=deltat)
    total_losses = []
    for i in range(len(results)):
        result = results[i]
        taus = varses[i][lasti_right:]
        lastt = sum(taus)
        tees = [0.0]
        t = 0.0
        while t < lastt:
            t += deltat
            tees.append(t)
        total_loss, scaled_losses, v_lasts = calc_losses(result, tees, lastt, targets)
        #if i == 0:
        if i == 0:
            print_list(v_lasts, 'v_lasts')
            print_list([sum(scaled_losses)] + scaled_losses, 'total/scaled_losses')

        total_losses.append(total_loss)

    # calculate the derivatives
    base_loss = total_losses[0]
    base_vars = varses[0]
    dldvs = []
    for i in range(1, len(total_losses)):
        if True:
            dloss = total_losses[i] - base_loss
        else:
            dloss = 0.0
        vars = varses[i]
        dv = vars[i-1] - base_vars[i-1]
        assert dv != 0.0
        dldvs.append(dloss/dv)
    return total_losses[0], dldvs

def update_vars(vars, dldvs, alpha):
    # update variables for gradient descent iteration
    new_vars = []
    last_speed_i = 2 * ((len(vars) + 1) // 3)
    for i in range(len(vars)):
        new_value = vars[i] - alpha * dldvs[i]
        if i >= last_speed_i:
            new_value = max(.1, new_value)
        new_vars.append(new_value)
    return new_vars

def main_test():

    def test_setup():
        model = keras.models.load_model('data/modelRnnFortieth15a')
        print(model.summary())

        left_edgeses = [
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0]
        ]
        right_edgeses = [
            [0.0, 1.0, 0.0],
            [0.0, .5, 1.0]
        ]
        tauses = [
            [1.01, 1.01],
            [1.11, 1.11]
        ]

        f_inputses = []
        lrs = []
        for i in range(len(left_edgeses)):
            left_edges = left_edgeses[i]
            right_edges = right_edgeses[i]
            taus = tauses[i]
            print(left_edges, right_edges, taus)
            lr = LR(left_edges, right_edges, taus, 1.0)
            lrs.append(lr)
            f_inputses.append((lr.f_left, lr.f_right,))
        tmax = max(map(sum, tauses))
        print(tmax)
        return f_inputses, tauses, lrs, model


    def test_LR():
        left_edges = [0.0, 1.0, -1.0, 0.0]
        right_edges = [0.0, 2.0, -1.0, 0.0]
        taus = [.5, .5, .5]
        lr = LR(left_edges, right_edges, taus, 1.0)
        print(lr.softclip(0.0))

        xes = []
        x = 0.0
        lefts = []
        rights = []
        xes = make_tees(taus, 0.025)
        lefts = list(map(lr.f_left, xes))
        rights = list(map(lr.f_right, xes))

        for i in range(len(xes)):
            print_list((xes[i], lefts[i], rights[i]), 'x, left, right')
        plt.plot(xes, lefts)
        plt.plot(xes, rights)
        plt.pause(.02)
        input('?')
        #print_list(xes, 'xes')
        #print_list(lefts_x, 'lefts_x')
        #print_list(rights_x, 'rights_x')

    def test_predict():
        model = keras.models.load_model('data/modelRnnFortieth15a')
        print(model.summary())
        left_edgeses = [
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0]
        ]
        right_edgeses = [
            [0.0, 1.0, 0.0],
            [0.0, .5, 1.0]
        ]
        tauses = [
            [1.01, 1.01],
            [1.11, 1.11]
        ]

        f_inputses = []
        lrs = []
        for i in range(len(left_edgeses)):
            left_edges = left_edgeses[i]
            right_edges = right_edgeses[i]
            taus = tauses[i]
            print(left_edges, right_edges, taus)
            lr = LR(left_edges, right_edges, taus, 1.0)
            lrs.append(lr)
            f_inputses.append((lr.f_left, lr.f_right,))
        tmax = max(map(sum, tauses))
        print(tmax)

        p = predict(model, f_inputses, tmax=tmax, deltat=0.025)
        print(p.shape)
        for i in range(p.shape[0]):
            tees = make_tees(tauses[i], 0.025)
            ll = len(tees)
            vxs = p[i, :ll, 0]
            vys = p[i, :ll, 1]
            dth = p[i, :ll, 2]
            lefts = list(map(lrs[i].f_left, tees))
            rights = list(map(lrs[i].f_right, tees))

            for i in range(len(tees)):
                print_list((tees[i], lefts[i], rights[i], vxs[i], vys[i], dth[i],), 'results')
            plt.plot(tees, lefts)
            plt.plot(tees, rights)
            plt.plot(tees, vxs)
            plt.plot(tees, vys)
            plt.plot(tees, dth)
            plt.pause(.02)
            input('?')

    def test_integrate():
        f_inputses, tauses, lrs, model = test_setup()

        tmax = max(map(sum, tauses))
        print(tmax)

        ps = predict(model, f_inputses, tmax=tmax, deltat=0.025)
        print(ps.shape)
        n = ps.shape[1]
        results = integrate_rates(ps, deltat=0.025)
        tees = np.linspace(0.0, (n -1) * 0.025, num=n)
        for i in range(len(results)):
            ll = len(tees)
            xs = results[i].xs
            ys = results[i].ys
            thetas = results[i].thetas
            lefts = list(map(lrs[i].f_left, tees))
            rights = list(map(lrs[i].f_right, tees))

            for i in range(len(tees)):
                print_list((tees[i], lefts[i], rights[i], xs[i], ys[i], thetas[i],), 'results')
            plt.plot(tees, lefts)
            plt.plot(tees, rights)
            plt.plot(tees, xs)
            plt.plot(tees, ys)
            plt.plot(tees, thetas)
            plt.pause(.02)
            input('?')

    def test_losses():
        class targets():
            x = 0.4
            y = 0.0
            theta = 1.0
            vx = 0.0
            vy = 0.0
            thetadot = 0.0
        f_inputses, tauses, lrs, model = test_setup()

        tmax = max(map(sum, tauses))
        print(tmax)

        ps = predict(model, f_inputses, tmax=tmax, deltat=0.025)
        print(ps.shape)
        n = ps.shape[1]
        tees = np.linspace(0.0, (n -1) * 0.025, num=n)
        results = integrate_rates(ps, deltat=0.025)
        for i in range(len(results)):
            result = results[i]
            print(result)
            taus = tauses[i]
            lastt = sum(taus)
            tees = [0.0]
            t = 0.0
            while t < lastt:
                t += .025
                tees.append(t)
            total_loss, scaled_losses = calc_losses(result, tees, lastt, targets)
            print('total_loss: {}'.format(total_loss))
            print_list(scaled_losses, 'scaled_losses')

    def test_vary_vars():
        left_edges = [0.0, .5, 1.0]
        right_edges = [0.0, -1.0, 0.0]
        taus = [.2, .4]
        varses = vary_vars(left_edges, right_edges, taus)
        print(varses)

    def test_get_dldv():
        left_edges = [0.0, -1.0, 0.0, 1.0, 0.0]
        right_edges = [0.0, 1.0, 0.0, 1.0, 0.0]
        taus = [.2, .2, .2, .2]
        class targets():
            x = 0.4
            y = 0.4
            theta = 1.0
            vx = 0.0
            vy = 0.0
            thetadot = 0.0
        model = keras.models.load_model('data/modelRnnFortieth15a')
        old_losses = None
        old_vars = None
        old_dldvs = None
        alpha = 0.001
        fig = plt.figure(figsize=(6,4))
        for count in range(200):
            print('\ncount= {}'.format(count))

            # plot
            lr = LR(left_edges, right_edges, taus, 1.0)
            tees = make_tees(taus, 0.025)
            lefts = list(map(lr.f_left, tees))
            rights = list(map(lr.f_right, tees))
            fig.clf()
            plt.plot(tees, lefts)
            plt.plot(tees, rights)
            plt.pause(.02)

            losses, dldvs = get_dldv(model, left_edges, right_edges, taus, targets, deltat=0.025)
            print_list(dldvs, 'dldvs')
            vars = left_edges + right_edges + taus
            if old_vars and losses > old_losses:
                if alpha < .000001:
                    break
                alpha /= 2.
                new_vars = update_vars(old_vars, old_dldvs, alpha=alpha)
            else:
                alpha *= 1.1
                new_vars = update_vars(vars, dldvs, alpha=alpha)
                old_vars = list(vars)
                old_dldvs = list(dldvs)
                old_losses = losses
            print_list(vars,     'vars    ')
            print_list(new_vars, 'new_vars')
            print('losses: {} old_losses: {} alpha: {}'.format(losses, old_losses, alpha))
            if losses < 2.0 and count > 10:
                break
            left_edges = new_vars[:len(left_edges)]
            right_edges = new_vars[len(left_edges):2*len(left_edges)]
            taus = new_vars[2*len(left_edges):]

    np.set_printoptions(precision=3, suppress=True)
    test_get_dldv()
    input('?')

main_test()

'''
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from nav_msgs.msg import Odometry
import tf
from functools import partial
import random
import time

D_TO_R = 3.1415926535 / 180. # degrees to radians
SEC_PER_STEP = 0.025
MAX_SPEED = 1.0
EPS = 1.0e-7
NITERS = 20
gamma = 0.9 # momentum factor
alpha = 0.001 # update rate

# TODO: move these to libpy
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

new_x = .15
new_y = .15
new_theta = 15.
target_twist.twist.linear.x = 0.0
target_twist.twist.linear.y = 0.0
target_twist.twist.angular.z = 0.0

target_pose.pose.position.x = new_x
target_pose.pose.position.y = new_y

qa_orig = q_to_array(current_pose.pose.orientation)
qa_rot = tf.transformations.quaternion_from_euler(0, 0, new_theta * D_TO_R)
qa_new = tf.transformations.quaternion_multiply(qa_rot, qa_orig)
q_new = array_to_q(qa_new)
target_pose.pose.orientation = q_new


# determine theta to new direction
delta_y = target_pose.pose.position.y - current_pose.pose.position.y
delta_x = target_pose.pose.position.x - current_pose.pose.position.x
delta_theta = math.atan2(delta_y, delta_x)
final_theta = poseTheta(current_pose, target_pose)

'''