# demo of python control library
import control
import scipy
from control.matlab import *  # MATLAB-like functions
from numpy.linalg import matrix_rank as rank, eig
import numpy as np
import matplotlib.pyplot as plt
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, \
 transform2d, pose3to2, DynamicStep

'''
https://github.com/python-control/Slycot/issues/15

What we do for testing in python-control is to use the following
set of commands to install slycot:

sudo apt-get install gfortran liblapack-dev
git clone https://github.com/python-control/Slycot.git slycot
cd slycot
sudo python setup.py install
'''
if __name__ == '__main__':
    # basic parameters
    lr_model = default_lr_model()
    (bxl, bxr, qx) = lr_model[0]
    (bol, bor, qo)= lr_model[2]
    '''
    qx = 1.0
    qo = 1.0
    bxl = 1.0
    bxr = 1.0
    bor = 1.0
    bol = -1.0
    '''

    omega0 = 0.0
    s0 = 0.11
    A = np.array((
            (-qx, 0, 0, 0, 0, -omega0 * s0),
            (omega0, 0, s0, 0, 0, -qx * s0),
            (0, 0, -qo, 0, 0, 0),
            (1, 0, 0, 0, 0, 0),
            (0, 1, 0, 0, 0, 0),
            (0, 0, 1, 0, 0, 0)
    ))
    B = np.array((
            (bxl, bxr),
            (0, 0),
            (bol, bor),
            (0, 0),
            (0, 0),
            (0, 0)
    ))

    '''
    C = np.identity(6)
    #D = ((0,0), (0,0), (0,0), (0,0), (0,0), (0,0))
    D = np.zeros((6,2))
    sys = control.ss(A, B, C, D)
    w, v = eig(A)
    print(gstr((w, v)))
    poles = np.array([-1,-1.2, -1.4, -1.6, -1.8, -2])
    K = scipy.signal.place_poles(A, B, poles,
     maxiter=100, method='KNV0')
    print(gstr(K.computed_poles))
    #T = A - B * K
    #print(gstr(eig(T)))
    '''

    Q = 10.0 * np.identity(6)
    R = np.identity(2)
    Kr, S, E = control.lqr(A, B, Q, R)
    print(gstr({'Kr': Kr, 'S': S, 'E': E}))
    T = A - B * Kr
    w, v = eig(T)
    print(gstr({'w': w, 'v': v}))
