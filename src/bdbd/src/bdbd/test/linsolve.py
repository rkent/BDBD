# tests speed of solving systems of linear equations

import numpy as np
import time
import matplotlib.pyplot as plt
counts = []
times = []

for count in range(2, 1024):
    a = np.random.rand(count, count)
    b = np.random.rand(count)
    start = time.time()
    x = np.linalg.solve(a, b)
    dt = time.time() - start
    counts.append(count)
    times.append(min(.040, dt))
    print('count: {} time: {}'.format(count, dt))

fig = plt.figure(figsize=(8,4))
plt.plot(counts, times)
plt.waitforbuttonpress()