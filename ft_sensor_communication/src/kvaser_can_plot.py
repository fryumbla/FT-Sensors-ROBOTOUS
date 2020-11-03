import numpy as np
import matplotlib.pyplot as plt

plt.axis([0, 30, -10, 10])
plt.ion()

for i in range(10):
    y = np.random.random()
    plt.scatter(i, y)
    plt.pause(0.05)

while True:
    plt.pause(0.05)
