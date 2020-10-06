from math import sin, cos, pi
from bdbd_common.geometry import D_TO_R, transform2d
from bdbd_common.utils import fstr
# Test of transfrom2D. See RKJ 2020-10-02 p 44 for example
'''
Adegrees = 14.04
Bdegrees = 45.0

xA = 6.4
yA = 9.6
thetaMDegrees = 90.0

AxM = 6.0
AyM = 3.0

BxM = 9.0
ByM = 8.0
'''

# free parameters
Adegrees = 0.0
Bdegrees = 0.0

xA = -0.13
yA = 0.0
thetaMDegrees = 0.0

AxM = 0.0
AyM = 0.0

BxM = -0.13
ByM = 0.0

thetaA = (thetaMDegrees - Adegrees) * D_TO_R
Atheta = Adegrees * D_TO_R
Btheta = Bdegrees * D_TO_R

poseA = (xA, yA, thetaA)
frameA = (AxM, AyM, Atheta)
frameB = (BxM, ByM, Btheta)

pA = transform2d(poseA, frameA, frameB)
print(fstr(pA))

pM = transform2d(poseA, frameA, (0., 0., 0.))
print(fstr(pM))
