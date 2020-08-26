# test circles in geometry
from bdbd.libpy.geometry import shortestPath

import math
D_TO_R = math.pi / 180. # degrees to radians

x = 3.0
rho = 0.5

# test cases from RKJ notebook 2020-08-26
phis = [60.5, 18.5, 27.0, -90.0]
yes = [2.0, 1.75, -2.0, -2.0]
for i in range(0, len(phis)):
    solution = shortestPath(rho, phis[i]* D_TO_R, x, yes[i])
    print('Solution: {}'.format(solution))
    print('gamma: {:6.2f}'.format(solution['gamma']/D_TO_R))
