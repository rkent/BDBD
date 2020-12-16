# newton-raphson iteration of motion equations

import numpy as np
import math
import time
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='12.9f', n_per_line=10)

class NewRaph():
    def __init__(self, lr_model=default_lr_model()):
        self.lr_model = lr_model

    def init(self, n, dt,
        pose0s=(0.0, 0.0, 0.0),
        twist0s=(0.0, 0.0, 0.0)
    ):
        self.n = n
        self.dt = dt
        self.pose0s = pose0s
        self.twist0s = twist0s
        # prep constants for calculations
        alr_model = np.array(self.lr_model)
        self.bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
        (bhxl, bhxr, qhx) = self.bhes[0]
        (bhyl, bhyr, qhy) = self.bhes[1]
        (bhol, bhor, qho) = self.bhes[2]
        print('(bhxl, bhxr, qhx): ' + estr((bhxl, bhxr, qhx)))
        print('(bhyl, bhyr, qhy): ' + estr((bhyl, bhyr, qhy)))
        print('(bhol, bhor, qho): ' + estr((bhol, bhor, qho)))
        (alphax, alphay, alphao) = 1.0 - np.array((qhx, qhy, qho))
        print('(alphax, alphay, alphao):' + estr((alphax, alphay, alphao)))
    
        # alpha ** j
        alphaxj = [1.0]
        alphayj = [1.0]
        alphaoj = [1.0]
        betaj = [dt]
        for i in range(1, n):
            alphaxj.append(alphaxj[i-1] * alphax)
            alphayj.append(alphayj[i-1] * alphay)
            alphaoj.append(alphaoj[i-1] * alphao)
            betaj.append(betaj[i-1] + dt * alphaoj[i])
        self.alphaxj = np.array(alphaxj)
        self.alphayj = np.array(alphayj)
        self.alphaoj = np.array(alphaoj)
        self.betaj = np.array(betaj)
        print('alphaxj:' + estr(self.alphaxj))
        print('alphayj:' + estr(self.alphayj))
        print('alphaoj:' + estr(self.alphaoj))
        print('betaj:' + estr(self.betaj))

        # first derivatives (Jacobians)
        '''
        dvxdl = np.zeros((n, n))
        dvydl = np.zeros((n, n))
        domdl = np.zeros((n, n))
        dvxdr = np.zeros((n, n))
        dvydr = np.zeros((n, n))
        domdr = np.zeros((n, n))
        dthdl = np.zeros((n, n))
        dthdr = np.zeros((n, n))
        for i in range(1, n):
            for k in range(1, i+1):
                dvxdl[i, k] = bhxl * alphaxj[i-k]
                dvydl[i, k] = bhyl * alphayj[i-k]
                domdl[i, k] = bhol * alphaoj[i-k]
                dvxdr[i, k] = bhxr * alphaxj[i-k]
                dvydr[i, k] = bhxl * alphayj[i-k]
                domdr[i, k] = bhor * alphaoj[i-k]
                dthdl[i, k] = bhol * betaj[i-k]
                dthdr[i, k] * bhor * betaj[i-k]
        print('dthdl:' + estr(dthdl))
        self.dvxdl = dvxdl
        self.dvydl = dvydl
        self.domdl = domdl
        self.dvxdr = dvxdr
        self.dvydr = dvydr
        self.domdr = domdr

        # constant matrices used in point hessian
        bkbm = np.zeros((n, n))
        bkaxm = np.zeros((n, n))
        bkaym = np.zeros((n, n))
        axkbm = np.zeros((n, n))
        aykbm = np.zeros((n, n))
        for jmk in range(0, n):
            for jmm in range(0, jmk):
                # j - m, j - k in formulas
                bkbm[jmk,jmm] = betaj[jmk] * betaj[jmm]
                bkaxm[jmk,jmm] = betaj[jmk] * alphaxj[jmm]
                bkaym[jmk,jmm] = betaj[jmk] * alphayj[jmm]
                axkbm[jmk,jmm] = alphaxj[jmk] * betaj[jmm]
                aykbm[jmk,jmm] = alphayj[jmk] * betaj[jmm]
        print('bkbm:' + estr(bkbm))
        print('betaj * betaj:'+  estr(np.matmul(self.betaj[:, np.newaxis], self.betaj[np.newaxis, :])))
        '''

    def poses(self, ls, rs):
        als = np.asarray(ls)
        ars = np.asarray(rs)
        #print('als:' + estr(als))
        (px0, py0, theta0) = self.pose0s
        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        (vxw0, vyw0, omega0) = self.twist0s
        n = self.n
        dt = self.dt
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        alphaoj = self.alphaoj
        betaj = self.betaj

        vx0 = vxw0 * math.cos(theta0) + vyw0 * math.cos(theta0)
        vy0 = -vxw0 * math.sin(theta0) + vyw0 * math.cos(theta0)

        # twist
        vxj = np.empty(n)
        vyj = np.empty(n)
        omegaj = np.empty(n)
        vxj[0] = vx0
        vyj[0] = vy0
        omegaj[0] = omega0
        bmotorxj = bhxl * als + bhxr * ars
        bmotoryj = bhyl * als + bhyr * ars
        bmotoroj = bhol * als + bhor * ars
        for i in range(1, n):
            vxj[i] = vx0 * alphaxj[i] + np.dot(alphaxj[i-1::-1], bmotorxj[1:i+1])
            vyj[i] = vy0 * alphayj[i] + np.dot(alphayj[i-1::-1], bmotoryj[1:i+1])
            omegaj[i] = omega0 * alphaoj[i] + np.dot(alphaoj[i-1::-1], bmotoroj[1:i+1])

        # pose
        pxj = np.empty(n)
        pyj = np.empty(n)
        thetaj = np.empty(n)
        pxj[0] = px0
        pyj[0] = py0
        thetaj[0] = theta0
        for i in range(1, n):
            thetaj[i] = theta0 + omega0 * (self.betaj[i] - dt) \
                + np.dot(self.betaj[i-1::-1], bmotoroj[1:i+1])
        cosj = np.cos(thetaj)
        sinj = np.sin(thetaj)
        vxcj = vxj * cosj
        vxsj = vxj * sinj
        vycj = vyj * cosj
        vysj = vyj * sinj

        vxwj = vxcj - vysj
        vywj = vxsj + vycj
        pxj[1:] = px0 + dt * np.cumsum(vxwj[1:])
        pyj[1:] = py0 + dt * np.cumsum(vywj[1:])

        # gradients

        dpxdl = np.zeros((n,n))
        dpydl = np.zeros((n,n))
        dpxdr = np.zeros((n,n))
        dpydr = np.zeros((n,n))
        elapsed_times = []

        for i in range(1, n):
            start = time.time()
            for k in range(1, i+1):
                doto = np.dot((-vxsj[k:i+1] - vycj[k:i+1]), betaj[:i+1-k])
                dotx = np.dot(cosj[k:i+1], alphaxj[:i+1-k])
                doty = np.dot(-sinj[k:i+1], alphayj[:i+1-k])
                dpxdl[i,k] = dt * (
                    +bhol * doto
                    +bhxl * dotx
                    +bhyl * doty
                )
                dpxdr[i,k] = dt * (
                    +bhor * doto
                    +bhxr * dotx
                    +bhyr * doty
                )
                doto = np.dot((vxcj[k:i+1] - vysj[k:i+1]), betaj[:i+1-k])
                dotx = np.dot(sinj[k:i+1], alphaxj[:i+1-k])
                doty = np.dot(cosj[k:i+1], alphayj[:i+1-k])
                dpydl[i,k] = dt * (
                    +bhol * doto
                    +bhxl * dotx
                    +bhyl * doty
                )
                dpydr[i,k] = dt * (
                    +bhor * doto
                    +bhxr * dotx
                    +bhyr * doty
                )
            elapsed_times.append(time.time() - start)

        print('elapsed: ' + gstr(elapsed_times))
        self.vxj = vxj
        self.vyj = vyj
        self.omegaj = omegaj
        self.pxj = pxj
        self.pyj = pyj
        self.thetaj = thetaj
        self.dpxdl = dpxdl
        self.dpydl = dpydl
        self.dpxdr = dpxdr
        self.dpydr = dpydr
        return (pxj, pyj, thetaj, vxj, vyj, omegaj)

if __name__ == '__main__':
    from bdbd_common.pathPlan2 import PathPlan
    import matplotlib.pyplot as plt

    pp = PathPlan()
    fig = plt.figure(figsize=(8,4))
    axis1 = None
    axis2 = None

    start_pose = [0.0, 0.0, 0.0]
    target_pose = [0.20, 0.10, 90. * D_TO_R] # x, y, theta
    start_twist = [0.0, 0.0, 0.0]
    target_twist = [0.0, 0.0, 0.0] # vx, vy, omega
    lr_start = (0.0, 0.0)
    cruise_v = 0.3

    pathPlan = pp.start2(start_pose, target_pose)
    print('path_plan:')
    for segment in pathPlan:
        print(fstr(segment))

    # estimate left, right to achieve the path
    speedPlan = pp.speedPlan(start_twist[0], cruise_v, target_twist[0], u=0.25)
    print('speed_plan:')
    for segment in speedPlan:
        print(fstr(segment))

    dt = 0.1
    frac = 0.0
    tt = 0.0
    vps = []

    vxr0 = start_twist[0] * math.cos(start_pose[2]) + start_twist[1] * math.sin(start_pose[2])
    vyr0 = -start_twist[0] * math.sin(start_pose[2]) + start_twist[1] * math.cos(start_pose[2])
    last_vx = vxr0
    last_omega = start_twist[2]

    vxres = [vxr0]
    vyres = [vyr0]
    omegas = [start_twist[2]]
    vvs = [pp.v(0.0)]
    vvs[0]['left'] = lr_start[0]
    vvs[0]['right'] = lr_start[1]
    lefts = [lr_start[0]]
    rights = [lr_start[1]]
    tt = 0.0
    tees = [tt]
    while True:
        tt += dt
        vv = pp.v(tt)
        vvs.append(vv)
        # vv gives vhat is in wheel frame. We need to convert to robot frame.
        vxres.append(vv['v'])
        vyres.append(vv['omega'] * pp.dwheel)
        omegas.append(vv['omega'])

        (left, right, last_vx, last_omega) = lr_est(vv['v'], vv['omega'], last_vx, last_omega, dt)
        lefts.append(left)
        rights.append(right)
        tees.append(tt)
        vv['left'] = left
        vv['right'] = right

        if vv['fraction'] > 0.9999:
            break
    for seg in vvs:
        print(estr(seg))

    last_vx = 0.0
    last_omega = 0.0

    #lr_model = [ [1., 1., 10.], [-.5, .5, 5.], [-8., 4., 10.]]
    lr_model = default_lr_model()
    nr = NewRaph(lr_model)
    n = len(lefts)
    nr.init(n, dt, pose0s=start_pose, twist0s=start_twist)
    start = time.time()
    (pxj, pyj, thetaj, vxj, vyj, omegaj) = nr.poses(lefts, rights)
    print('elapsed time: {}', time.time() - start)
    print('vxj:' + estr(vxj))
    print('vyj:' + estr(vyj))
    print('omegaj:' + estr(omegaj))
    print('thetaj:' + estr(thetaj))
    print('pxj:' + estr(pxj))
    print('pyj:' + estr(pyj))

    # get numerical estimate of derivative for single values
    _lefts = lefts[:]
    _rights = rights[:]
    k = 3
    eps = 0.0001
    _rights[k] += eps
    start = time.time()
    (_pxj, _pyj, _thetaj, _vxj, _vyj, _omegaj) = nr.poses(_lefts, _rights)
    print('elapsed time: {}', time.time() - start)
    dpxj = (_pxj - pxj) / eps
    dpyj = (_pyj - pyj) / eps
    print('dpxj:' + estr(dpxj))
    print('dpxdr:' + estr(nr.dpxdr[:,k]))

    '''
    print('dpxdl:' + estr(nr.dpxdl, n_per_line=18))
    print('dpxdr:' + estr(nr.dpxdr))
    print('dpydr:' + estr(nr.dpydr))
    '''

    fig.clf()
    plt1 = fig.add_subplot(121)
    #plt1.axis([0.0, tfPath.lrs[-1]['t'], -1.5, 1.5])
    plt2 = fig.add_subplot(122)

    if axis1 is not None:
        plt1.axis(axis1)
    if axis2 is not None:
        plt2.axis(axis2)
    else:
        plt2.axis('equal')

    #plt2.axis([-0.00, 0.40, -0.05, 0.40])

    #plt1.plot(tees, txs)np.concatenate(([0.0], lefts)))
    plt1.plot(tees, lefts)
    plt1.plot(tees, rights)
    plt1.plot(tees, omegaj)

    plt2.plot(pxj, pyj)

    plt.pause(0.001)
    if axis1 is None:
        axis1 = plt1.axis()
    if axis2 is None:
        axis2 = plt2.axis()

    plt.waitforbuttonpress()