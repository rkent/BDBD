%% basic parameters


pkg load control
qx = 1.0
qo = 1.0
bxl = 1.0
bxr = 1.0
bor = 1.0
bol = -1.0
#omega0 = -0.925
omega0 = 0.0
s0 = 0.11
A = [
        -qx, 0, 0, 0, 0, -omega0 * s0;
        omega0, 0, s0, 0, 0, -qx * s0;
        0, 0, -qo, 0, 0, 0;
        1, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0
]
B = [
        bxl, bxr;
        0, 0;
        bol, bor;
        0, 0;
        0, 0;
        0, 0
]

C = eye(6)
#D = ((0,0), (0,0), (0,0), (0,0), (0,0), (0,0))
D = zeros(6,2)
sys = ss(A, B, C, D)
K = place(A,B,[-1,-1.1,-1.2,-1.3,-1.4,-1.5])
T = A - B * K
[DD,VV] = eig(T)
