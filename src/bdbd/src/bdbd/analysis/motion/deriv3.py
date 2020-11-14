# checking of derivative calculations in NewRaph

import numpy as np 
import time
from math import sin, cos
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R

def estr(a):
    return fstr(a, fmat='25.22f')


# using formulas
def beta(i):
    sum = 0.0
    for j in range(i+1):
        sum += alphao**i
    return dt * sum

dr = .1
dt = 0.05
steps = 2
n = steps

# prep constants for calculations
lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
alr_model = np.array(lr_model)
bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
(bhxl, bhxr, qhx) = bhes[0]
(bhyl, bhyr, qhy) = bhes[1]
(bhol, bhor, qho) = bhes[2]
alphas = 1.0 - np.array((qhx, qhy, qho))
(alphax, alphay, alphao) = alphas
print('(alphax, alphay, alphao):' + estr((alphax, alphay, alphao)))
print('bhes:' + estr(bhes))

# calculation from NewRaph3
alphaxj = [1.0]
alphayj = [1.0]
alphaoj = [1.0]
betaj = [dt]
for i in range(1, n):
    alphaxj.append(alphaxj[i-1] * alphax)
    alphayj.append(alphayj[i-1] * alphay)
    alphaoj.append(alphaoj[i-1] * alphao)
    betaj.append(betaj[i-1] + dt * alphaoj[i])

print('(alphaxj, alphayj, alphaoj, betaj): ' + estr((alphaxj, alphayj, alphaoj, betaj)))

eps = 0.0001

# values of (1) from NewRaph for different eps combination, base eps = 0.001
# (left, right): [ (1.0, 1.0) (1.0 + eps, 1.0) (1.0, 1.0 + eps), (1.0 + eps, 1.- + eps)]
vx1s = [0.1, 0.100005, 0.100005, 0.10001]
vy1s = [0.0, -0.000005, 0.000005, 0.0]
omega1s = [0.45, 0.449995, 0.450050000000, 0.450045000000]
px1s = [0.0049987344283926541527, 0.0049989900177008694468, 0.0049989784587078766459, 0.0049992340486938856986]
py1s = [0.000112490508, 0.000112244946, 0.000112758567, 0.000112513005]
theta1s = [0.0225, 0.022499750000, 0.022502500000, 0.022502250000]

# derivatives using px values
vx1 = vx1s[0]
vy1 = vy1s[0]
omega1 = omega1s[0]
px1 = px1s[0]
py1 = py1s[0]
theta1 = theta1s[0]
sin1 = sin(theta1)
cos1 = cos(theta1)

d2pxdl1dr1 = dt * (
     -dt * dt * bhol * bhor * (vx1 * cos1 + vy1 * sin1)
     -sin1 * dt * (bhol * bhxr + bhxl * bhor)
     -cos1 * dt * (bhol * bhyr + bhyl * bhor)
)

(pxj, pxjpe, pxkpe, pxjpekpy) = px1s
dpxj = (pxjpe - pxj) / eps
dpxj2 = (pxjpekpy - pxkpe) / eps
d2px = (dpxj2 - dpxj) / eps
print('dpxj: ' + estr(dpxj) + 'dpxj2: ' + estr(dpxj2) + ' d2px: ' + estr(d2px))
print('d2pxdl1dr1: ' + estr(d2pxdl1dr1))

