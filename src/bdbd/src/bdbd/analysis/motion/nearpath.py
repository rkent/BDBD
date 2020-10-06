# calculate the near path for approaching a pose

import math
from bdbd_common.geometry import twoArcPlans, ccwPath, D_TO_R 
from bdbd_common.utils import fstr

''' extreme example
theta_degrees = -90
theta = theta_degrees * D_TO_R
X = 12.0
Y = -8.0
'''

'''
# ccw-cw, 2020-09-12 pp 22
theta_degrees = 22.0
theta = theta_degrees * D_TO_R
X = 6.3
Y = 5.7
'''

# cw-ccw, 2020-09-13 pp 24
'''
theta_degrees = -21.0
theta = theta_degrees * D_TO_R
X = 6.0
Y = -4.7
'''

# vary at will
theta_degrees = 90
theta = theta_degrees * D_TO_R
X = -1.0
Y = 3.0

plans = twoArcPlans(X, Y, theta)
for plan in plans:
    print('plan:')
    for s in plan:
        print(fstr(s))
