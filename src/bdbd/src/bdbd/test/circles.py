# development of circular movement paths
# see R. Kent James notebook circa 2020-08-16

import math
import numpy as np
np.set_printoptions(precision=4)

D_TO_R = math.pi / 180.
TWOPI = 2.0 * math.pi

# point we want to move to

phiDegrees = 27. # final orientation relative to current robot orientation
x = 3 # center of new location, in rear_wheels coordinates
y = -2
rho = .5
phi = phiDegrees * D_TO_R

# radius of circle of movement

# motion circles from start, in rear_wheels
# sc : start circle
sc_ccw = np.array([0.0, rho])
sc_cw = np.array([0.0, -rho])

# fc: end_circle
fc_ccw = np.array([x - rho * math.sin(phi), y + rho * math.cos(phi)])
fc_cw = np.array([x + rho * math.sin(phi), y - rho * math.cos(phi)])

A = np.array([0., 0.])
B = np.array([x, y])
solutions = []
for start_dir in range(0, 2):
    for end_dir in range(0, 2):
        C = sc_ccw if start_dir == 0 else sc_cw
        D = fc_ccw if end_dir == 0 else fc_cw
        D_C = D - C
        a = D_C[0]
        b = D_C[1]
        theta = math.atan2(b, a)
        tsq = a**2 + b**2
        if start_dir != end_dir and tsq - 4. * rho **2 < 0.:
            length = None
            print('dir: {} {} invalid'.format(start_dir, end_dir))
            #solutions.push({start_dir: start_dir, end_dir: end_dir, length: length})
        else:
            if start_dir == end_dir:
                psi = None
                alpha = None
                ssq = tsq
                beta = theta if start_dir == 0  else -theta
            else:
                ssq = tsq - (2 * rho)**2
                psi = math.acos(2. * rho / math.sqrt(tsq))
                alpha = psi - theta if start_dir == 0 else psi + theta
                beta = math.pi/ 2. - alpha
            s = math.sqrt(ssq)
            beta = beta % TWOPI

            E = rho * np.array([math.sin(beta), 1. - math.cos(beta)])
            if start_dir == 1:
                E[1] = -E[1]
            if end_dir == 0:
                F = np.array([D[0] + rho * math.sin(beta), D[1] - rho * math.cos(beta)])
            else:
                F = np.array([D[0] - rho * math.sin(beta), D[1] + rho * math.cos(beta)])
            
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
            c1 = rho * beta
            c2 = rho * gamma
            length = s + c1 + c2
            print('dir: {} {} length {}'.format(start_dir, end_dir, length))
            print(A, B, C, D, E, F)
            print('s {}, theta {}, phi {}, psi {}, alpha {}, beta {}, gamma {}, c1 {}, c2 {}: '
                .format(s, theta, phi, psi, alpha, beta, gamma, c1, c2))
