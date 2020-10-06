# just a scratchpad for test calculations
import math

ls = .565
le = .603
l = .5 * (ls + le)
vs = .3
ve = 0.0

dt = 0.25

vsq = vs**2 + 2. * (l - ls)*(ve - vs) / dt
v = math.sqrt(vsq) if vsq > 0.0 else ve
print(v, vsq)