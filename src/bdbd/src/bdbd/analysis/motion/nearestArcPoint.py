from bdbd_common.geometry import HALFPI, nearestArcPoint
from bdbd_common.utils import fstr

center = ( 0.200, -0.025 )
alpha = -.927
rho = 0.125
thetaStart = .927 + HALFPI
point = (0.220, 0.095)


print(fstr(nearestArcPoint(center, rho, thetaStart, alpha, point)))