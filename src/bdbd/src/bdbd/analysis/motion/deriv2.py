# checking of derivative calculations in NewRaph

import numpy as np 
import time
from math import sin, cos
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='15.12f')

'''
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

    print('twists:' + estr(twists))
    print('thetas: '  + estr(thetas))
    print('vrxes:' + estr(vrxes))
    print('vryes:' + estr(vryes))
    print('pxes:' + estr(pxes))
    print('pyes:' + estr(pyes))
    return (pxes, pyes, vxes, vyes, thetas)
'''

# using formulas
def beta(i):
    sum = 0.0
    for j in range(i+1):
        sum += alphao**i
    return dt * sum

def dpxdr(i, k, vxes, vyes, thetas):
    print('vxes, vyes, thetas:' + estr((vxes, vyes, thetas)))
    dotx = 0.0
    doty = 0.0
    doto1 = 0.0
    doto2 = 0.0
    stheta = []
    alphas = []
    for j in range(k, i+1):
        vx = vxes[j]
        vy = vyes[j]
        theta = thetas[j]
        stheta.append(sin(theta))
        alphas.append(alphax**(j-k))
        doto1 += -vx * sin(theta) * beta(j-k)
        doto2 += -vy * cos(theta) * beta(j-k)
        dotx += cos(theta) * alphax**(j-k)
        doty += -sin(theta) * alphay**(j-k)
    print(estr({
        'doto1': dt * bhor *doto1,
        'doto2': dt * bhor * doto2,
        'dotx': dt * bhxr * dotx,
        'doty': dt * bhyr * doty}))
    print(estr({'sin(theta)': stheta}))
    print('alphas: ' + estr(alphas))
    sum = dt * (
        +bhor * (doto1 + doto2)
        +bhxr * dotx
        +bhyr * doty
    )
    return sum

dr = .1
dt = 0.05
steps = 2

# prep constants for calculations
lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
alr_model = np.array(lr_model)
bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
(bhxl, bhxr, qhx) = bhes[0]
(bhyl, bhyr, qhy) = bhes[1]
(bhol, bhor, qho) = bhes[2]
alphas = 1.0 - np.array((qhx, qhy, qho))
(alphax, alphay, alphao) = alphas
#print('(alphax, alphay, alphao):' + gstr((alphax, alphay, alphao)))
#print('bhes:' + gstr(bhes))
eps = 0.0
px1 = 0.0
vx1 = 0.0
vy1 = 0.0
theta1 = 0.0

for count in range(8):
    vx = 0.10 + 0.05 *eps
    vy = .05 * eps
    omega = 0.45 + .5 * eps
    theta = 0.0225 + 0.025 * eps
    vrx = vx * cos(theta) - vy * sin(theta)

    if eps == 0.0:
        px1 = vrx * dt
        eps = 0.1
        vxes = [0.0, vx]
        vyes = [0.0, vy]
        vx1 = vx
        vr1 = vy
        theta1 = theta
        thetas = [0.0, theta]
        dpdr = dpxdr(1, 1, vxes, vyes, thetas)
        print('calculated dpdr:' + estr(dpdr))
        # calculate individual terms using derivative calculations
        vx_coss = -dt * vx * sin(theta) * dt * bhor
        vxs_cos = dt * cos(theta) * bhxr
        vy_sins = - dt * vy * cos(theta) * dt * bhor
        vys_sin = - dt * sin(theta) * bhyr
        print(estr({'vxs_cos': vxs_cos, 'vx_coss': vx_coss, 'vys_sin': vys_sin, 'vy_sins':vy_sins}))
        print(estr({'sum': vx_coss + vxs_cos + vy_sins + vys_sin}))
        print('')
    else:
        dtde = dt / eps
        px2 = vrx * dt
        dpdr = (px2 - px1) / eps
        print(estr({'eps': eps, 'vrx': vrx, 'px1': px1, 'px2': px2, 'dpdr': dpdr}))
        # individual terms
        vxs = vx - vx1
        vys = vy - vy1
        coss = cos(theta) - cos(theta1)
        sins = sin(theta) - sin(theta1)
        vxs_cos = vxs * cos(theta1)
        vx_coss = vx1 * coss
        vys_sin = -vys * sin(theta1)
        vy_sins = -vy1 * sins
        vxs_coss = vxs * coss
        vys_sins = -vys * sins
        print(fstr({'vxs_cos': vxs_cos*dtde, 'vx_coss': vx_coss*dtde, 'vxs_coss': vxs_coss*dtde,
                    'vys_sin': vys_sin*dtde, 'vy_sins': vy_sins*dtde, 'vys_sins': vys_sins*dtde},
                    fmat='15.12f'))
        print(estr({'sum': (vxs_cos + vx_coss + vxs_coss + vys_sin + vy_sins + vys_sins)*dtde}))

        eps /= 10


