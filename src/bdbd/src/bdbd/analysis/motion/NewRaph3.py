# newton-raphson iteration of motion equations

import numpy as np
import math
import time
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='25.22f', n_per_line=10)

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
        #print('(bhxl, bhxr, qhx): ' + estr((bhxl, bhxr, qhx)))
        #print('(bhyl, bhyr, qhy): ' + estr((bhyl, bhyr, qhy)))
        #print('(bhol, bhor, qho): ' + estr((bhol, bhor, qho)))
        (alphax, alphay, alphao) = 1.0 - np.array((qhx, qhy, qho))
        #print('(alphax, alphay, alphao):' + estr((alphax, alphay, alphao)))
    
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
        #print('alphaxj:' + estr(self.alphaxj))
        #print('alphayj:' + estr(self.alphayj))
        #print('alphaoj:' + estr(self.alphaoj))
        #print('betaj:' + estr(self.betaj))

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

        # Hessians (2nd partial derivatives)
        d2pxdldl = np.zeros((n, n, n))
        d2pxdldr = np.zeros((n, n, n))
        d2pxdrdr = np.zeros((n, n, n))
        d2pydldl = np.zeros((n, n, n))
        d2pydldr = np.zeros((n, n, n))
        d2pydrdr = np.zeros((n, n, n))

        elapsed_times = []
        for i in range(1, n):
            start = time.time()

            # gradients
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

        # 2nd derivatives (Hessians)
        for j in range(1, n):
            start = time.time()
            vxwdt = vxwj[j] * dt
            vywdt = vywj[j] * dt
            sdt = sinj[j] * dt
            cdt = cosj[j] * dt
            for k in range(1, j + 1):
                betaljk = betaj[j-k] * bhol
                betarjk = betaj[j-k] * bhor
                alphaxljk = alphaxj[j-k] * bhxl
                alphaxrjk = alphaxj[j-k] * bhxr
                alphayljk = alphayj[j-k] * bhyl
                alphayrjk = alphayj[j-k] * bhyr
                for m in range(1, j + 1):
                    betaljm = betaj[j-m] * bhol
                    betarjm = betaj[j-m] * bhor
                    alphaxljm = alphaxj[j-m] * bhxl
                    alphaxrjm = alphaxj[j-m] * bhxr
                    alphayljm = alphaxj[j-m] * bhyl
                    alphayrjm = alphaxj[j-m] * bhyr

                    sumxll = (
                        -vxwdt * betaljk * betaljm
                        +sdt * (-betaljk * alphaxljm -alphaxljk * betaljm)
                        +cdt * (-betaljk * alphayljm -alphayljk * betaljm)
                    )
                    sumxlr = (
                        -vxwdt * betaljk * betarjm
                        +sdt * (-betaljk * alphaxrjm -alphaxljk * betarjm)
                        +cdt * (-betaljk * alphayrjm -alphayljk * betarjm)
                    )
                    sumxrr = (
                        -vxwdt * betarjk * betarjm
                        +sdt * (-betarjk * alphaxrjm -alphaxrjk * betarjm)
                        +cdt * (-betarjk * alphayrjm -alphayrjk * betarjm)
                    )
                    sumyll = (
                        -vywdt * betaljk * betaljm
                        +sdt * (-betaljk * alphayljm -alphayljk * betaljm)
                        +cdt * (betaljk * alphayljm +alphayljk * betaljm)
                    )
                    sumylr = (
                        -vywdt * betaljk * betarjm
                        +sdt * (-betaljk * alphayrjm -alphayljk * betarjm)
                        +cdt * (betaljk * alphayrjm +alphayljk * betarjm)
                    )
                    sumyrr = (
                        -vywdt * betarjk * betarjm
                        +sdt * (-betarjk * alphayrjm -alphayrjk * betarjm)
                        +cdt * (betarjk * alphayrjm +alphayrjk * betarjm)
                    )

                    for i in range(j, n):
                        #print('i,j,k,m', i, j, k, m)
                        d2pxdldl[i, k, m] += sumxll
                        d2pxdldr[i, k, m] += sumxlr
                        d2pxdrdr[i, k, m] += sumxrr
                        d2pydldl[i, k, m] += sumyll
                        d2pydldr[i, k, m] += sumylr
                        d2pydrdr[i, k, m] += sumyrr

                elapsed_times.append(time.time() - start)

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
        self.d2pxdldl = d2pxdldl
        self.d2pxdldr = d2pxdldr
        self.d2pxdrdr = d2pxdrdr
        self.d2pydldl = d2pydldl
        self.d2pydldr = d2pydldr
        self.d2pydrdr = d2pydrdr
        self.elapsed_times = elapsed_times
        return (pxj, pyj, thetaj, vxj, vyj, omegaj)

if __name__ == '__main__':
    from bdbd_common.pathPlan2 import PathPlan
    import matplotlib.pyplot as plt

    dt = 0.05
    frac = 0.0
    tt = 0.0
    n = 50
    vps = []
    #lr_model = default_lr_model()
    lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    nr = NewRaph(lr_model)
    lefts = n * [1.0]
    rights = n * [1.0]
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]

    n = len(lefts)
    nr.init(n, dt, pose0s=start_pose, twist0s=start_twist)
    start = time.time()
    (pxj, pyj, thetaj, vxj, vyj, omegaj) = nr.poses(lefts, rights)
    end = time.time()

    '''
    print('dpxdl:' + gstr(nr.dpxdl, fmat='25.22f', n_per_line=18))
    print('dpxdr:' + gstr(nr.dpxdr, fmat='25.22f', n_per_line=18))
    print('dpydl:' + gstr(nr.dpydl, fmat='25.22f', n_per_line=18))
    print('dpydr:' + gstr(nr.dpydr, fmat='25.22f', n_per_line=18))
    print('d2pxdldl' + gstr(nr.d2pxdldl, fmat='25.22f', n_per_line=18))
    print('d2pxdldr' + gstr(nr.d2pxdldr, fmat='25.22f', n_per_line=18))
    print('d2pxdrdr' + gstr(nr.d2pxdrdr, fmat='25.22f', n_per_line=18))
    print('d2pydldl' + gstr(nr.d2pydldl, fmat='25.22f', n_per_line=18))
    print('d2pydldr' + gstr(nr.d2pydldr, fmat='25.22f', n_per_line=18))
    print('d2prdrdr' + gstr(nr.d2pydrdr, fmat='25.22f', n_per_line=18))
    print('vxj:' + gstr(vxj, fmat='25.22f'))
    print('vyj:' + gstr(vyj, fmat='25.22f'))
    print('omegaj:' + gstr(omegaj, fmat='25.22f'))
    print('thetaj:' + fstr(thetaj, fmat='25.22f'))
    print('pxj:' + fstr(pxj, fmat='25.22f'))
    print('pyj:' + fstr(pyj, fmat='25.22f'))
    print('sin(theta):' + fstr(np.sin(thetaj), fmat='25.22f'))
    print('cos(theta):' + fstr(np.cos(thetaj), fmat='25.22f'))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj), fmat='25.22f'))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj), fmat='25.22f'))
    '''

    print('elapsed time', end-start)
    #print('elapsed times: ', gstr(nr.elapsed_times))

    '''
    # get numerical estimate of derivative for single values
    j = 1
    k = 2
    eps = 0.0001

    _lefts = lefts[:]
    _rights = rights[:]
    _lefts[j] += eps
    (pxjpe, pyjpe, thetaj, vxj, vyj, omegaj) = nr.poses(_lefts, _rights)
    print('+l')
    print('vxj:' + gstr(vxj, fmat='25.22f'))
    print('vyj:' + gstr(vyj, fmat='25.22f'))
    print('omegaj:' + gstr(omegaj, fmat='25.22f'))
    print('thetaj:' + fstr(thetaj, fmat='25.22f'))
    print('pxj:' + fstr(pxjpe, fmat='25.22f'))
    print('pyj:' + fstr(pyjpe, fmat='25.22f'))
    print('sin(theta):' + fstr(np.sin(thetaj), fmat='25.22f'))
    print('cos(theta):' + fstr(np.cos(thetaj), fmat='25.22f'))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj), fmat='25.22f'))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj), fmat='25.22f'))

    _lefts = lefts[:]
    _rights = rights[:]
    _rights[k] += eps
    (pxkpe, pykpe, thetaj, vxj, vyj, omegaj) = nr.poses(_lefts, _rights)
    print('+r)')
    print('vxj:' + gstr(vxj, fmat='25.22f'))
    print('vyj:' + gstr(vyj, fmat='25.22f'))
    print('omegaj:' + gstr(omegaj, fmat='25.22f'))
    print('thetaj:' + fstr(thetaj, fmat='25.22f'))
    print('pxj:' + fstr(pxkpe, fmat='25.22f'))
    print('pyj:' + fstr(pykpe, fmat='25.22f'))
    print('sin(theta):' + fstr(np.sin(thetaj), fmat='25.22f'))
    print('cos(theta):' + fstr(np.cos(thetaj), fmat='25.22f'))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj), fmat='25.22f'))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj), fmat='25.22f'))

    _lefts = lefts[:]
    _rights = rights[:]
    _rights[k] += eps
    _lefts[j] += eps
    (pxjpekpe, pyjpekpe, thetaj, vxj, vyj, omegaj) = nr.poses(_lefts, _rights)
    print('+l+r')
    print('vxj:' + gstr(vxj, fmat='25.22f'))
    print('vyj:' + gstr(vyj, fmat='25.22f'))
    print('omegaj:' + gstr(omegaj, fmat='25.22f'))
    print('thetaj:' + fstr(thetaj, fmat='25.22f'))
    print('pxj:' + fstr(pxjpekpe, fmat='25.22f'))
    print('pyj:' + fstr(pyjpekpe, fmat='25.22f'))
    print('sin(theta):' + fstr(np.sin(thetaj), fmat='25.22f'))
    print('cos(theta):' + fstr(np.cos(thetaj), fmat='25.22f'))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj), fmat='25.22f'))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj), fmat='25.22f'))

    dpxj = (pxjpe - pxj) / eps
    dpyj = (pyjpe - pyj) / eps
    d2pxjk = (pxjpekpe + pxj - pxjpe - pxkpe) / eps**2
    d2pyjk = (pyjpekpe + pyj - pyjpe - pykpe) / eps**2
    print('dpxj:' + estr(dpxj))
    print('dpxdr:' + estr(nr.dpxdr[:,k]))
    print('d2pxjk:' + estr(d2pxjk))
    print('d2pyjk:' + estr(d2pyjk))
    print('d2pxdldr[i, ', j, k, ']')
    print(gstr(nr.d2pxdldr[:, j, k], fmat='25.22f', n_per_line=18))

'''