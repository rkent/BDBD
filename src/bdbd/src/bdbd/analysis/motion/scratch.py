# just a scratchpad for test calculations
from numpy.lib.financial import nper
import numpy as np 
from numpy.random import default_rng
import time
from math import sin, cos
from bdbd_common.utils import fstr, gstr

doto = -0.000590388324
dotx =  1.498291828682  
doty = -0.067469392412
bhor = 0.5
bhxr = 0.05
bhyr = 0.05
dt = 0.05
dd = dt * (bhor * doto + bhxr * dotx + bhyr * doty)
print(dd)

def doit():
    print(meglobal)

meglobal = 1.0
doit()
