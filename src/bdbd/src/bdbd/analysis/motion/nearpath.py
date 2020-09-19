# calculate the near path for approaching a pose

import math

D_TO_R = math.pi / 180.
theta_degrees = 21.7
theta = theta_degrees * D_TO_R
X = 6.3
Y = 5.7
A = 1.0 - math.cos(theta)
B = Y * (1. + math.cos(theta)) - X * math.sin(theta)
C = - (X**2 + Y**2) / 2.

print(A, B, C)
#rho = (-B + math.sqrt(B**2 - 4. * A * C)) / (2. * A)
#rhom = (-B - math.sqrt(B**2 - 4. * A * C)) / (2. * A)
try:
    rho = 2. * C / (-B - math.sqrt(B**2 - 4. * A * C))
except ZeroDivisionError:
    rho = 1.e10

# calculate diagram values
g = rho * math.sin(theta)
a = X + g
b = rho * (1. + math.cos(theta)) - Y
beta = math.atan2(a , b) / D_TO_R
gamma = beta - theta_degrees
print(rho, g, a, b, beta, gamma)

