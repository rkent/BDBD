# newton-raphson iteration of motion equations

import numpy as np
import math
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='15.12f')

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
        print('(bhxl, bhxr, qhx): ' + gstr((bhxl, bhxr, qhx)))
        print('(bhyl, bhyr, qhy): ' + gstr((bhyl, bhyr, qhy)))
        print('(bhol, bhor, qho): ' + gstr((bhol, bhor, qho)))
        (alphax, alphay, alphao) = 1.0 - np.array((qhx, qhy, qho))
        print('(alphax, alphay, alphao):' + gstr((alphax, alphay, alphao)))
    
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
        print('alphaxj:' + gstr(self.alphaxj, fmat='15.12f'))
        print('alphayj:' + gstr(self.alphayj, fmat='15.12f'))
        print('alphaoj:' + gstr(self.alphaoj, fmat='15.12f'))
        print('betaj:' + gstr(self.betaj, fmat='15.12f'))

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
        print('dthdl:' + gstr(dthdl))
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
        print('bkbm:' + gstr(bkbm))
        print('betaj * betaj:'+  gstr(np.matmul(self.betaj[:, np.newaxis], self.betaj[np.newaxis, :])))
        '''

    def poses(self, ls, rs):
        als = np.asarray(ls)
        ars = np.asarray(rs)
        #print('als:' + fstr(als))
        (px0, py0, theta0) = self.pose0s
        (bhxl, bhxr, qhx) = self.bhes[0]
        (bhyl, bhyr, qhy) = self.bhes[1]
        (bhol, bhor, qho) = self.bhes[2]
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
            #print('alphaoj[i-1::-1]' + fstr(self.alphaoj[i-1::-1]))
            #print('bmotoroj[1:i+1]: ' + fstr(bmotoroj[1:i+1]))
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

        for i in range(1, n):
            for k in range(1, i+1):
                doto = np.dot((-vxsj[k:i+1] - vycj[k:i+1]), betaj[:i+1-k])
                dotx = np.dot(cosj[k:i+1], alphaxj[:i+1-k])
                doty = np.dot(-sinj[k:i+1], alphayj[:i+1-k])
                '''
                if i == 2:
                    print('-vxsj[k:i+1]:' + fstr(-vxsj[k:i+1], fmat='15.12f'))
                    print('-vycj[k:i+1]:' + fstr(-vycj[k:i+1], fmat='15.2f'))
                    print('betaj[i-k::-1]:' + fstr(betaj[i-k::-1], fmat='15.12f'))
            
                    print('cosj[k:i+1]:' + fstr(cosj[k:i+1], fmat='15.12f'))
                    print('alphaxj[i-k::-1]:' + fstr(alphaxj[i-k::-1], fmat='15.12f'))
                    print('-sinj[k:i+1]:' + fstr(-sinj[k:i+1], fmat='15.12f'))
                    print('alphayj[i-k::-1]:' + fstr(alphayj[i-k::-1], fmat='15.12f'))
                    print('doto:' + estr(doto))
                    print('dotx:' + estr(dotx))
                    print('doty:' + estr(doty))
                    print()
                '''
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

    start_pose = [0.0, 0.0, 0.0]
    target_pose = [0.15, 0.05, 0. * D_TO_R] # x, y, theta
    start_twist = [0.0, 0.0, 0.0]
    target_twist = [0.0, 0.0, 0.0] # vx, vy, omega
    lr_start = (0.0, 0.0)
    cruise_v = 0.3

    dt = 0.05
    frac = 0.0
    tt = 0.0
    vps = []
    #lr_model = default_lr_model()
    lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    nr = NewRaph(lr_model)
    lefts = [1.0, 1.0, 1.0, 1.0]
    rights = [1.0, 1.0, 1.0, 1.0]

    n = len(lefts)
    nr.init(n, dt, pose0s=start_pose, twist0s=start_twist)
    (pxj, pyj, thetaj, vxj, vyj, omegaj) = nr.poses(lefts, rights)
    print('vxj:' + gstr(vxj, fmat='15.12f'))
    print('vyj:' + gstr(vyj, fmat='15.12f'))
    print('omegaj:' + gstr(omegaj, fmat='15.12f'))
    print('thetaj:' + fstr(thetaj, fmat='15.12f'))
    print('pxj:' + fstr(pxj, fmat='15.12f'))
    print('pyj:' + fstr(pyj, fmat='15.12f'))
    print('sin(theta):' + fstr(np.sin(thetaj), fmat='15.12f'))
    print('cos(theta):' + fstr(np.cos(thetaj), fmat='15.12f'))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj), fmat='15.12f'))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj), fmat='15.12f'))
    print('dpxdr:' + gstr(nr.dpxdr, fmat='15.12f', n_per_line=18))
    print('dpxdr type:' + fstr(nr.dpxdr.dtype))
    print(' ')

    # get numerical estimate of derivative for single values
    '''
    _lefts = lefts[:]
    _rights = rights[:]
    k = 1
    eps = 0.001
    _rights[k] += eps
    (_pxj, _pyj, _thetaj, _vxj, _vyj, _omegaj) = nr.poses(_lefts, _rights)
    dpxj = (_pxj - pxj) / eps
    dpyj = (_pyj - pyj) / eps
    print('dpxj:' + gstr(dpxj, fmat='15.12f', n_per_line=18))

    print('_vxj:' + gstr(_vxj, fmat='15.12f'))
    print('_vyj:' + gstr(_vyj, fmat='15.12f'))
    print('_omegaj:' + gstr(_omegaj, fmat='15.12f'))
    print('_thetaj:' + fstr(_thetaj, fmat='15.12f'))
    print('_pxj:' + fstr(_pxj, fmat='15.12f'))
    print('_pyj:' + fstr(_pyj, fmat='15.12f'))
    print('_sin(theta):' + fstr(np.sin(_thetaj), fmat='15.12f'))
    print('_cos(theta):' + fstr(np.cos(_thetaj), fmat='15.12f'))
    print('_vxw:' + fstr(_vxj * np.cos(_thetaj) - _vyj * np.sin(_thetaj), fmat='15.12f'))
    print('_vyw:' + fstr(_vxj * np.sin(_thetaj) + _vyj * np.cos(_thetaj), fmat='15.12f'))
'''
