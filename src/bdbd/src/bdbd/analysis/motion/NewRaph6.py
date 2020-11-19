# newton-raphson iteration of motion equations

import numpy as np
import math
import time
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='10.7g', n_per_line=10)

class NewRaph():
    def __init__(self, n, dt, lr_model=default_lr_model()):
        self.lr_model = lr_model
        self.n = n
        self.dt = dt

        # prep constants for calculations
        alr_model = np.array(self.lr_model)
        self.bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
        (_, _, qhx) = self.bhes[0]
        (_, _, qhy) = self.bhes[1]
        (_, _, qho) = self.bhes[2]
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

    def poses(self, ls, rs,
        start_pose=(0.0, 0.0, 0.0),
        start_twist=(0.0, 0.0, 0.0),
        details=False
    ):
        als = np.asarray(ls)
        ars = np.asarray(rs)
        self.als = als
        self.ars = ars
        #print('als:' + estr(als))
        (px0, py0, theta0) = start_pose
        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        (vxw0, vyw0, omega0) = start_twist
        n = self.n
        dt = self.dt
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        alphaoj = self.alphaoj

        # initial robot velocities
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

        if details:
            print(estr({'alphaoj[n-2::-1]': alphaoj[n-2::-1]}))
            print(estr({'bmotoroj[1:n]': bmotoroj[1:n]}))
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

        # intermediate values as vectors
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

        # intermediate results
        self.cosj = cosj
        self.sinj = sinj
        self.vxcj = vxcj
        self.vxsj = vxsj
        self.vycj = vycj
        self.vysj = vysj
        self.vxwj = vxwj
        self.vywj = vywj

        self.vxj = vxj
        self.vyj = vyj
        self.omegaj = omegaj
        self.pxj = pxj
        self.pyj = pyj
        self.thetaj = thetaj
        return (pxj, pyj, thetaj, vxj, vyj, omegaj)

    def gradients(self):
        # gradients

        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        n = self.n
        dt = self.dt
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        betaj = self.betaj

        cosj = self.cosj
        sinj = self.sinj
        vxcj = self.vxcj
        vxsj = self.vxsj
        vycj = self.vycj
        vysj = self.vysj

        dpxdl = np.zeros((n,n))
        dpydl = np.zeros((n,n))
        dpxdr = np.zeros((n,n))
        dpydr = np.zeros((n,n))

        for i in range(1, n):
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
        self.dpxdl = dpxdl
        self.dpydl = dpydl
        self.dpxdr = dpxdr
        self.dpydr = dpydr

        return (dpxdl, dpxdr, dpydl, dpydr)

    def seconds(self):
        # second partial derivatives

        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        n = self.n
        dt = self.dt
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        betaj = self.betaj

        cosj = self.cosj
        sinj = self.sinj
        vxwj = self.vxwj
        vywj = self.vywj

        d2pxdldl = np.zeros((n, n, n))
        d2pxdldr = np.zeros((n, n, n))
        d2pxdrdr = np.zeros((n, n, n))
        d2pydldl = np.zeros((n, n, n))
        d2pydldr = np.zeros((n, n, n))
        d2pydrdr = np.zeros((n, n, n))

        # This could be vectorized, but instead I do it discretely to more closely
        # match the C++ version which is what we will actually use.
        for j in range(1, n):
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

        self.d2pxdldl = d2pxdldl
        self.d2pxdldr = d2pxdldr
        self.d2pxdrdr = d2pxdrdr
        self.d2pydldl = d2pydldl
        self.d2pydldr = d2pydldr
        self.d2pydrdr = d2pydrdr
        return (d2pxdldl, d2pxdldr, d2pxdrdr, d2pydldl, d2pydldr, d2pydrdr)

    def loss(self,
            target_pose=(0.0, 0.0, 0.0),
            target_twist=(0.0, 0.0, 0.0),
            target_lr = (0.0, 0.0),
            Wmax=1.0e-4,
            Wjerk=1.0e-4,
            Wback=1.0e-4,
            mmax=1.0,
            details=False
    ):
        self.target_pose = target_pose
        self.target_twist = target_twist
        self.target_lr = target_lr
        self.Wmax = Wmax
        self.Wjerk = Wjerk
        self.Wback = Wback
        self.mmax = mmax
        return self.reloss(details=details)

    def reloss(self, details=False):
        target_pose = self.target_pose
        target_twist = self.target_twist
        target_lr = self.target_lr
        Wmax = self.Wmax
        Wjerk = self.Wjerk
        Wback = self.Wback
        mmax = self.mmax
        # given pose calculations, determine the loss
        vxj = self.vxj
        vyj = self.vyj
        omegaj = self.omegaj
        pxj = self.pxj
        pyj = self.pyj
        thetaj = self.thetaj

        lefts = self.als
        rights = self.ars

        # values requiring summing over i
        sumMax = 0.1 * Wmax * (
            np.power(lefts, 10.0).sum() +np.power(rights, 10.0).sum()
        ) / mmax ** 10

        # backing term
        sumBack = 0.1 * Wback * np.power((lefts + rights).clip(max=0.0), 10).sum()

        ldiff = lefts[1:] - lefts[:-1]
        rdiff = rights[1:] - rights[:-1]
        sumJerk = 0.5 * Wjerk * (np.square(ldiff).sum() + np.square(rdiff).sum())

        # values based on final targets
        vals = np.asarray([
            pxj[-1]
            , pyj[-1]
            , thetaj[-1]
            , vxj[-1]
            , vyj[-1]
            , omegaj[-1]
            , lefts[-1]
            , rights[-1]
        ])
        targets = np.concatenate([target_pose, target_twist, target_lr])
        #targets = np.concatenate([target_pose, target_twist[:1], target_lr])
        sumTargets = 0.5 * np.square(vals - targets).sum()
        loss = sumMax + sumJerk + sumTargets + sumBack
        if details:
            print('target losses: ' + estr(0.5 * np.square(vals - targets)))
            print(estr({'loss': loss, 'sumMax': sumMax, 'sumJerk': sumJerk, 'sumTargets': sumTargets, 'sumBack': sumBack}))
            print(fstr({'vals': vals}, fmat='15.12g'))
            print(fstr({'targets': targets}))
        self.lossValue = loss
        return loss

    def jacobian(self):
        # the 1st derivative of the loss function
        vxj = self.vxj
        vyj = self.vyj
        omegaj = self.omegaj
        pxj = self.pxj
        pyj = self.pyj
        thetaj = self.thetaj

        (pxt, pyt, thetat) = self.target_pose
        (vxt, vyt, omegat) = self.target_twist
        (leftt, rightt) = self.target_lr
        dpxdl = self.dpxdl
        dpydl = self.dpydl
        dpxdr = self.dpxdr
        dpydr = self.dpydr

        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        alphaoj = self.alphaoj
        betaj = self.betaj
        Wmax = self.Wmax
        Wjerk = self.Wjerk
        Wback = self.Wback
        mmax = self.mmax

        lefts = self.als
        rights = self.ars
        leftsp9 = np.power(lefts / mmax, 9)
        rightsp9 = np.power(rights / mmax, 9)
        lprsp9 = np.power((lefts + rights).clip(max=0.0), 9)
        n = len(lefts)

        dlefts = np.zeros([n])
        drights = np.zeros([n])

        for k in range(1, n):
            dlefts[k] = (
                +(vxj[-1] - vxt) * bhxl * alphaxj[n-1-k]
                +(vyj[-1] - vyt) * bhyl * alphayj[n-1-k]
                +(omegaj[-1] - omegat) * bhol * alphaoj[n-1-k]
                +(thetaj[-1] - thetat) * bhol * betaj[n-1-k]
                +(pxj[-1] - pxt) * dpxdl[-1, k]
                +(pyj[-1] - pyt) * dpydl[-1, k]
                +Wmax * leftsp9[k] / mmax
                +Wback * lprsp9[k]
                +Wjerk * (2 * lefts[k] -lefts[k-1] -lefts[min(k+1, n-1)])
            )
            drights[k] = (
                +(vxj[-1] - vxt) * bhxr * alphaxj[n-1-k]
                +(vyj[-1] - vyt) * bhyr * alphayj[n-1-k]
                +(omegaj[-1] - omegat) * bhor * alphaoj[n-1-k]
                +(thetaj[-1] - thetat) * bhor * betaj[n-1-k]
                +(pxj[-1] - pxt) * dpxdr[-1, k]
                +(pyj[-1] - pyt) * dpydr[-1, k]
                +Wmax * rightsp9[k]
                +Wback * lprsp9[k]
                +Wjerk * (2 * rights[k] -rights[k-1] -rights[min(k+1, n-1)])
            )
        # TODO: check this
        dlefts[-1] += (lefts[-1] - leftt)
        drights[-1] += (rights[-1] - rightt)
        self.dlefts = dlefts
        self.drights = drights
        return (dlefts, drights)

    def hessian(self):
        # second derivative of the loss function
        pxj = self.pxj
        pyj = self.pyj
        (pxt, pyt, _) = self.target_pose
        dpxdl = self.dpxdl
        dpydl = self.dpydl
        dpxdr = self.dpxdr
        dpydr = self.dpydr
        (bhxl, bhxr, _) = self.bhes[0]
        (bhyl, bhyr, _) = self.bhes[1]
        (bhol, bhor, _) = self.bhes[2]
        alphaxj = self.alphaxj
        alphayj = self.alphayj
        alphaoj = self.alphaoj
        betaj = self.betaj

        Wmax = self.Wmax
        Wjerk = self.Wjerk
        Wback = self.Wback
        mmax = self.mmax

        lefts = self.als
        rights = self.ars
        d2pxdldl = self.d2pxdldl
        d2pxdldr = self.d2pxdldr
        d2pxdrdr = self.d2pxdrdr
        d2pydldl = self.d2pydldl
        d2pydldr = self.d2pydldr
        d2pydrdr = self.d2pydrdr
        n = len(lefts) - 1

        # We'll define this as 0 -> n-1 are lefts[1:], n -> 2n-1 are rights[1:]
        hess = np.empty([2*n, 2*n])

        # values that vary with each k, m value
        deltapxn = pxj[-1] - pxt
        deltapyn = pyj[-1] - pyt
        for i in range(0, 2*n):
            k = i % n + 1

            kleft = (i < n)
            if kleft:
                dpxdu = dpxdl[n, k]
                dpydu = dpydl[n, k]
                dvxdu = alphaxj[n-k] * bhxl
                dvydu = alphayj[n-k] * bhyl
                domdu = alphaoj[n-k] * bhol
                dthdu = betaj[n-k] * bhol
            else:
                dpxdu = dpxdr[n, k]
                dpydu = dpydr[n, k]
                dvxdu = alphaxj[n-k] * bhxr
                dvydu = alphayj[n-k] * bhyr
                domdu = alphaoj[n-k] * bhor
                dthdu = betaj[n-k] * bhor

            for j in range(0, 2*n):
                m = j % n + 1
                mleft = (j < n)
                if mleft:
                    dpxds = dpxdl[n, m]
                    dpyds = dpydl[n, m]
                    dvxds = alphaxj[n-m] * bhxl
                    dvyds = alphayj[n-m] * bhyl
                    domds = alphaoj[n-m] * bhol
                    dthds = betaj[n-m] * bhol
                    
                    if kleft:
                        d2px = d2pxdldl[n, k, m]
                        d2py = d2pydldl[n, k, m]
                    else:
                        # note d2pxdrdl[i,j] = d2pxdldr[j,i]
                        d2px = d2pxdldr[n, m, k]
                        d2py = d2pydldr[n, m, k]
                else:
                    dpxds = dpxdr[n, m]
                    dpyds = dpydr[n, m]
                    dvxds = alphaxj[n-m] * bhxr
                    dvyds = alphayj[n-m] * bhyr
                    domds = alphaoj[n-m] * bhor
                    dthds = betaj[n-m] * bhor
                    if kleft:
                        d2px = d2pxdldr[n, k, m]
                        d2py = d2pydldr[n, k, m]
                    else:
                        d2px = d2pxdrdr[n, k, m]
                        d2py = d2pydrdr[n, k, m]
                hess[i, j] = (
                    deltapxn * d2px + dpxdu * dpxds +
                    deltapyn * d2py + dpydu * dpyds +
                    dvxdu * dvxds + dvydu * dvyds + domdu * domds + dthdu * dthds
                )

        # values that require k == m
        for i in range(0, 2*n):
            k = i % n + 1
            kleft = (i < n)
            # max term
            hess[i, i] += 9 * (Wmax / mmax**2) * (lefts[k]**8 if kleft else rights[k]**8)
            # back term
            if (lefts[k] + rights[k]) < 0.0:
                hess[i, i] += 9 * Wback * (lefts[k] + rights[k])**8
            # motor target value
            if k == n:
                hess[i, i] += 1.0
            # jerk term
            hess[i, i] += 2 *Wjerk
            if k > 1:
                hess[i, i-1] -= Wjerk
            if k == n:
                hess[i, i] -= Wjerk
            else:
                hess[i, i+1] -= Wjerk
        
        self.hess = hess
        return hess

    def dloss_dleft(self, j, eps=1.e-3):
        # numerical estimate of loss derivative at left[j]
        base_als = self.als.copy()

        lefts = base_als.copy()
        lefts[j] += eps
        nr.poses(lefts, self.ars)
        loss_plus = nr.reloss()

        lefts = base_als.copy()
        lefts[j] -= eps
        nr.poses(lefts, self.ars)
        loss_minus = nr.reloss()
        self.als = base_als

        dloss = 0.5 * (loss_plus - loss_minus) / eps
        return dloss

    def d2loss_dl_dl(self, k, eps=0.0001):
        # numerical estimate of second derivative of loss  dl dl
        base_als = self.als.copy()
        n = len(self.als)

        d2lossj = [0.0]
        for j in range(1, n):
            lefts = base_als.copy()
            lefts[k] += eps
            self.als = lefts
            #dlossp = self.dloss_dleft(j, eps)
            nr.poses(lefts, self.ars)
            nr.gradients()
            nr.jacobian()
            dlossp = self.dlefts[j]
            pxp = self.pxj[-1]

            lefts = base_als.copy()
            lefts[k] -= eps
            self.als = lefts
            #dlossm = self.dloss_dleft(j, eps)
            nr.poses(lefts, self.ars)
            nr.gradients()
            nr.jacobian()
            dlossm = self.dlefts[j]
            pxm = self.pxj[-1]
            d2lossj.append(0.5 * (dlossp - dlossm) / eps)
            #print(estr({'pxp': pxp, 'pxm': pxm, 'pxp - pxm': pxp - pxm}))
            print(estr(({'dlossp': dlossp, 'dlossm': dlossm, 'dlossp-dlossm': dlossp-dlossm, 'wjerk': self.Wjerk})))
        self.als = base_als

        return d2lossj
    
    def dloss_dright(self, j, eps=0.0001):
        # numerical estimate of loss derivative at left[j]
        base_ars = self.ars.copy()

        rights = base_ars.copy()
        rights[j] += eps
        nr.poses(self.als, rights)
        loss_plus = nr.reloss()

        rights = base_ars.copy()
        rights[j] -= eps
        nr.poses(self.als, rights)
        loss_minus = nr.reloss()
        self.ars = base_ars

        dloss = 0.5 * (loss_plus - loss_minus) / eps

        return dloss

if __name__ == '__main__':
    from bdbd_common.pathPlan2 import PathPlan
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8,4))
    axis1 = None
    axis2 = None

    dt = 0.05
    lr_model = default_lr_model()
    #lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]
    target_pose = [0.001, 0.000, 0.0]
    target_twist = [0.0, 0.0, 0.0]
    cruise_v = 0.3
    lr_start = (0.0, 0.0)
    gauss_iters = 0
    nr_iters = 10
    Wmax = 1.e-6
    Wjerk = 1.e-3
    Wback = .1
    NRstart = 0.1
    NRfact = 2.0
    maxSlew = 0.1

    avgImprove = 0.0
    targetImprove = 1.e-3
    pp = PathPlan()
    pathPlan = pp.start2(start_pose, target_pose)
    print('path_plan:')
    for segment in pathPlan:
        print(fstr(segment))

    # estimate left, right to achieve the path
    speedPlan = pp.speedPlan(start_twist[0], cruise_v, target_twist[0], u=0.25)
    print('speed_plan:')
    for segment in speedPlan:
        print(fstr(segment))

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

    lefts = lefts[:10]
    rights = rights[:10]
    tees = tees[:10]
    n = len(lefts)
    nr = NewRaph(n, dt, lr_model)
    # gradient descent iteration
    eps = 0.5
    last_loss = 1.e10
    last_lefts = None
    last_rights = None
    last_dlefts = None
    last_drights = None

    for count in range(gauss_iters + nr_iters):
        start = time.time()
        (pxj, pyj, thetaj, vxj, vyj, omegaj) = nr.poses(lefts, rights,
            start_pose=start_pose, start_twist=start_twist)
        #print('pose time:', time.time() - start)

        start = time.time()
        (dpxdl, dpxdr, dpydl, dpydr) = nr.gradients()
        #print('gradients time:', time.time() - start)
        #print(len(lefts), len(dpxdl))

        start = time.time()
        loss = nr.loss(mmax=1.0, target_pose=target_pose, Wmax=Wmax, Wjerk=Wjerk, Wback=Wback, details=True)
        #print('loss time:', time.time() - start)
        #print('loss:', loss)

        start = time.time()
        (dlefts, drights) = nr.jacobian()
        #print('jacobian time:', time.time() - start)

        '''
        start = time.time()
        (d2pxdldl, d2pxdldr, d2pxdrdr, d2pydldl, d2pydldr, d2pydrdr) = nr.seconds()
        print('seconds time:', time.time() - start)
        '''

        fig.clf()
        plt1 = fig.add_subplot(121)
        #plt1.axis([0.0, tfPath.lrs[-1]['t'], -1.5, 1.5])
        plt2 = fig.add_subplot(122)

        '''
        if axis1 is not None:
            plt1.axis(axis1)
        if axis2 is not None:
            plt2.axis(axis2)
        else:
        '''
        plt2.axis('equal')

        plt1.plot(tees, lefts)
        plt1.plot(tees, rights)
        plt1.plot(tees, thetaj)
        plt1.plot(tees, vxj)

        plt2.plot(pxj, pyj)

        plt.pause(0.001)
        if axis1 is None:
            axis1 = plt1.axis()
        if axis2 is None:
            axis2 = plt2.axis()

        # update lefts, rights
        print(fstr({'count': count, 'eps': eps, 'loss': loss, 'last_loss': last_loss,
            'lmax': np.amax(lefts), 'rmax': np.amax(rights)}, fmat='15.12f'))
        if loss < last_loss or last_lefts is None:
            improve = loss / last_loss
            avgImprove = 0.25 * (3*avgImprove + improve)
            if avgImprove > (1.0 - targetImprove):
                print('Done')
                break
            last_loss = loss
            last_lefts = lefts.copy()
            last_rights = rights.copy()
            last_dlefts = dlefts.copy()
            last_drights = drights.copy()
            # gradient descent
            if count < gauss_iters:
                for i in range(1, n):
                    lefts[i] -= eps*dlefts[i]
                    rights[i] -= eps*drights[i]
                eps *= 1.1
            else:
                if count == gauss_iters:
                    eps = NRstart
                (d2pxdldl, d2pxdldr, d2pxdrdr, d2pydldl, d2pydldr, d2pydrdr) = nr.seconds()
                hess = nr.hessian()
                print('Using Newton-Raphson!')
                b = np.concatenate([-nr.dlefts[1:], -nr.drights[1:]])
                deltax = np.linalg.solve(nr.hess, b)

                # limit slew rate
                slew = np.amax(np.absolute(deltax))
                eps = min(eps, maxSlew / slew)
                nhess = len(lefts) - 1
                lefts[1:] += eps * deltax[:nhess]
                rights[1:] += eps * deltax[nhess:]
                eps = min(1.0, NRfact*eps)
                print('deltax:' + estr(deltax))

        else:
            print('Loss got worse')
            eps *= .5 / NRfact
            for i in range(1, n):
                lefts[i] = last_lefts[i] - eps * last_dlefts[i]
                rights[i] = last_rights[i] - eps * last_drights[i]
            
        #print('lefts:' + estr(lefts))
        #print('rights:' + estr(rights))
    plt.waitforbuttonpress()
    print('pxj:' + estr(pxj))
    print('pyj:' + estr(pyj))
    print('thetaj:' + estr(thetaj))
    print('vxj:' + estr(vxj))
    print('vyj:' + estr(vyj))
    print('omegaj:' + estr(omegaj))
    print('lefts:' + estr(lefts))
    print('rights:' + estr(rights))
    #print('dlefts:' + fstr(dlefts, fmat='18.15g', n_per_line=10))
    #print('drights:' + fstr(drights, fmat='18.15g', n_per_line=10))
    #print('hessian:' + fstr(nr.hess, fmat='18.15g', n_per_line=10))

    '''
    print('dpxdl:' + gstr(nr.dpxdl, fmat='18.15f', n_per_line=10))
    print('dpxdr:' + gstr(nr.dpxdr, fmat='18.15f', n_per_line=10))
    print('dpydl:' + gstr(nr.dpydl, fmat='18.15f', n_per_line=10))
    print('dpydr:' + gstr(nr.dpydr, fmat='18.15f', n_per_line=10))
    print('d2pxdldl' + gstr(nr.d2pxdldl, fmat='18.15f', n_per_line=10))
    print('d2pxdldr' + gstr(nr.d2pxdldr, fmat='18.15f', n_per_line=10))
    print('d2pxdrdr' + gstr(nr.d2pxdrdr, fmat='18.15f', n_per_line=10))
    print('d2pydldl' + gstr(nr.d2pydldl, fmat='18.15f', n_per_line=10))
    print('d2pydldr' + gstr(nr.d2pydldr, fmat='18.15f', n_per_line=10))
    print('d2prdrdr' + gstr(nr.d2pydrdr, fmat='18.15f', n_per_line=10))
    print('sin(theta):' + fstr(np.sin(thetaj),fmat='18.15f', n_per_line=10))
    print('cos(theta):' + fstr(np.cos(thetaj),fmat='18.15f', n_per_line=10))
    print('vxw:' + fstr(vxj * np.cos(thetaj) - vyj * np.sin(thetaj),fmat='18.15f', n_per_line=10))
    print('vyw:' + fstr(vxj * np.sin(thetaj) + vyj * np.cos(thetaj),fmat='18.15f', n_per_line=10))
    '''

    # numerical estimate of dlefts
    # get numerical estimate of derivative for single values

    dlefts_n = [0.0]
    dpxdl_n = [0.0]
    dpydl_n = [0.0]
    dthetadl_n = [0.0]
    dvxdl_n = [0.0]
    dvydl_n = [0.0]
    domegadl_n = [0.0]
    for j in range(1, n):
        dloss = nr.dloss_dleft(j, eps=1.e-3)
        dlefts_n.append(dloss)

    drights_n = [0.0]
    dpxdr_n = [0.0]
    dpydr_n = [0.0]
    dthetadr_n = [0.0]
    dvxdr_n = [0.0]
    dvydr_n = [0.0]
    domegadr_n = [0.0]
    for j in range(1, n):
        dloss = nr.dloss_dright(j, eps=0.0001)

    print('dpxdl_n:' + estr(dpxdl_n))
    print('dpxdl:' + estr(nr.dpxdl))
    print('dpydl_n:' + estr(dpydl_n))
    print('dpydl:' + estr(nr.dpydl))
    print('dlefts_n:' + fstr(dlefts_n, fmat='18.15g', n_per_line=10))
    print('dlefts:' + fstr(nr.dlefts, fmat='18.15g', n_per_line=10))    
    print('drights_n:' + fstr(drights_n, fmat='18.15g', n_per_line=10))
    print('drights:' + fstr(nr.drights, fmat='18.15g', n_per_line=10))
    d2loss = []
    for k in range(1, len(lefts)):
        d2loss.append(nr.d2loss_dl_dl(k, eps=1.0e-3))
    print('d2lossj:' + fstr(d2loss, fmat='18.15g', n_per_line=10))
    '''
    #nr.poses(nr.als, nr.ars)
    #nr.reloss()
    #nr.gradients()
    #(dlefts, drights) = nr.jacobian()
    print('hessian:' + fstr(nr.hess, fmat='18.15g', n_per_line=10))
print('d2pxdldl' + gstr(nr.d2pxdldl, fmat='18.15f', n_per_line=10))
'''

# look at 5,5
'''
eps = 1.0e-3
base_als = nr.als.copy()

lefts = base_als.copy()
lefts[5] += eps
#dlossp = self.dloss_dleft(j, eps)
nr.poses(lefts, nr.ars, details=True)
lossp = nr.reloss(True)
nr.gradients()
nr.jacobian()
dlossp = nr.dlefts[5]
pxp = nr.pxj[-1]
pyp = nr.pyj[-1]

lefts[5] -= eps
nr.poses(lefts, nr.ars, details=True)
lossc = nr.reloss(True)
nr.gradients()
nr.jacobian()
dlossc = nr.dlefts[5]
pxc = nr.pxj[-1]
pyc = nr.pyj[-1]

lefts[5] -= eps
nr.poses(lefts, nr.ars, details=True)
lossm = nr.reloss(True)
nr.gradients()
nr.jacobian()
dlossm = nr.dlefts[5]
pxm = nr.pxj[-1]
pym = nr.pyj[-1]
print(estr({'dlossp': dlossp, 'dlossc': dlossc, 'dlossm': dlossm}))
print(estr({'d1': 0.5 * (lossp - lossm)/eps, 'd2': (-lossp + 2*lossc - lossm)/eps**2}))
print(estr({'pxloss': .5 *(pxc-.01)**2, 'p1x': 0.5 * (pxp - pxm) / eps, 'p2x': (-pxp + 2*pxc - pxm)/eps**2}))
print(estr({'pyloss': 0.5 * pyc**2, 'p1y': 0.5 * (pyp - pym) / eps, 'p2y': (-pyp + 2*pyc - pym)/eps**2}))
'''