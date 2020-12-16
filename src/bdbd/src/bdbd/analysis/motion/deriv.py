# checking of derivative calculations in NewRaph

from numpy.lib.financial import nper
import numpy as np 
from numpy.random import default_rng
import time
from math import sin, cos
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='15.12f')

def pes(lefts, rights):
    vxes = [0.0]
    vyes = [0.0]
    vrxes = [0.0]
    vryes = [0.0]
    omegas = [0.0]
    twists = [vxes, vyes, omegas]
    pxes = [0.0]
    pyes = [0.0]
    thetas = [0.0]
    for i in range(1, steps):
        # manual
        for w in range(3):
            ves = twists[w]
            ves.append(ves[-1] * alphas[w] + bhes[w][0] * lefts[i] + bhes[w][1] * rights[i])
        thetas.append(thetas[-1] + dt * twists[2][-1])
        vx = twists[0][-1]
        vy = twists[1][-1]
        theta = thetas[-1]
        vrxes.append(vx * cos(theta) - vy * sin(theta))
        vryes.append(vx * sin(theta) + vy * cos(theta))
        pxes.append(pxes[-1] + dt * vrxes[-1])
        pyes.append(pyes[-1] + dt * vryes[-1])

    '''
    print('twists:' + estr(twists))
    print('thetas: '  + estr(thetas))
    print('vrxes:' + estr(vrxes))
    print('vryes:' + estr(vryes))
    print('pxes:' + estr(pxes))
    print('pyes:' + estr(pyes))
    '''

    return (pxes, pyes, vxes, vyes, thetas)

# using formulas
def beta(i):
    sum = 0.0
    for j in range(i+1):
        sum += alphao**i
    return dt * sum

def dpxdr(i, k, vxes, vyes, thetas):
    print('vxes, vyes, thetas:' + estr((vxes, vyes, thetas)))
    doto = 0.0
    dotx = 0.0
    doty = 0.0
    stheta = []
    alphas = []
    for j in range(k, i+1):
        vx = vxes[j]
        vy = vyes[j]
        theta = thetas[j]
        stheta.append(sin(theta))
        alphas.append(alphax**(j-k))
        doto += (vx * sin(theta) *  beta(j-k) -vy * cos(theta)) * beta(j-k)
        dotx += cos(theta) * alphax**(j-k)
        doty += -sin(theta) * alphay**(j-k)
    print(estr({'doto': doto, 'dotx': dotx, 'doty': doty}))
    print(estr({'sin(theta)': stheta}))
    print('alphas: ' + estr(alphas))
    sum = dt * (
        +bhor * doto
        +bhxr * dotx
        +bhyr * doty
    )
    return sum

dr = .1
dt = 0.05
steps = 3

# prep constants for calculations
lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
alr_model = np.array(lr_model)
bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
(bhxl, bhxr, qhx) = bhes[0]
(bhyl, bhyr, qhy) = bhes[1]
(bhol, bhor, qho) = bhes[2]
alphas = 1.0 - np.array((qhx, qhy, qho))
(alphax, alphay, alphao) = alphas
print('(alphax, alphay, alphao):' + gstr((alphax, alphay, alphao)))
print('bhes:' + gstr(bhes))
eps = .1
lefts = steps * [1.0]
rights = steps * [1.0]
(px1s, py1s, vx1s, vy1s, theta1s) = pes(lefts, rights)

for count in range(6):
    lefts = steps * [1.0]
    rights = steps * [1.0]
    rights[1] += eps
    (px2s, py2s, vx2s, vy2s, theta2s) = pes(lefts, rights)
    dPX2dR1 = (px2s[1] - px1s[1]) / eps
    print(estr({'eps': eps, 'dPX2DR1': dPX2dR1}))
    eps /= 10

print(estr('dpdr' + estr(dpxdr(1, 1, vx1s, vy1s, theta1s))))


