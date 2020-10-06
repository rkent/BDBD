# calculate the near path for approaching a pose

import math
from bdbd_common.geometry import shortestPath, D_TO_R 
from bdbd_common.utils import fstr

'''
# both cw 2020-08-21 pp 10
theta_degrees = -112
theta = theta_degrees * D_TO_R
X = 9.8
Y = -14.7
approach_rho = 2.0

# cw then ccw, 2020-08-26 pp 17
theta_degrees = 27.
theta = theta_degrees * D_TO_R
X = 12.0
Y = -8.0
approach_rho = 2.0

# ccw then cw, 2020-08-26 pp 16
theta_degrees = 18.5
theta = theta_degrees * D_TO_R
X = 12.0
Y = 7.0
approach_rho = 2.0
'''

# both ccw, 2020-08-26b pp 15
'''
theta_degrees = 60.5
theta = theta_degrees * D_TO_R
X = 12.0
Y = 8.0
approach_rho = 2.0
'''

# vary at will
theta_degrees = -90.0
theta = theta_degrees * D_TO_R
X = 12.0
Y = 8.0
approach_rho = 2.0

plan = shortestPath(X, Y, theta, approach_rho)
for s in plan:
    print(fstr(s))
