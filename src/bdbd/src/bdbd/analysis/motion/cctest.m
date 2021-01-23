%% basic parameters


pkg load control
qx = 7.929
qo = 8.464
bxl = 1.258
bxr = 1.378
bor = 7.624
bol = -7.659
#omega0 = -0.925
omega0 = 0.517
s0 = 0.155
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

K =   [ 
      0.269      2.15    -0.508     0.495     0.869     -2.76,
      0.104      -1.4     0.474     0.869    -0.495      2.15,
]
  
#C = eye(6)
#D = ((0,0), (0,0), (0,0), (0,0), (0,0), (0,0))
#D = zeros(6,2)
#sys = ss(A, B, C, D)
#K2 = place(A,B,[-4.9, -1.8, -1.6, -1.4, -1.2, -1.0])
fact = 0.95
base = -2.0
EE = []
for i = 1:6
  EE(i) = base
  base = base * fact
endfor
K2 = place(A, B, EE)

T = A - B * K
[DD,VV] = eig(T)
