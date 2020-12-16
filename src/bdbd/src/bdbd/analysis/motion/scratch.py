# just a scratchpad for test calculations
from numpy.lib.financial import nper
import numpy as np 
from numpy.random import default_rng
import time
from math import sin, cos
from bdbd_common.utils import fstr, gstr
tp = -0.00111230739817
tc = -0.00106991938304
tm = -0.0010275313679

print((-tp -tm +2*tc)/(1.e-3)**2)
print('tp-tc', tp-tc, 'tc-tm', tc-tm)

alpha = [0.0122518596654208, 0.0212410881855422, 0.0368257423466405, 0.0638449069809995,  0.110688118899098,     0.191900344832,         0.33269824,             0.5768,                  1 ]
bmotor = [0.00292872850023364, -0.00116699925899113, -0.000150659061121022, -0.000102798320922463, -0.000324244574766243, -0.000441262410214323, -0.000181578006905556, -0.00152929510021145, -5.82561059489067e-06 ]

sum = 0.0
for i in range(len(alpha)):
    sum += alpha[i] * bmotor[i]

print('omegan', sum)