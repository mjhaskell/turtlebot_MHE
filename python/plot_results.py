import numpy as np
import matplotlib.pyplot as plt
from extractdata import *

lms = np.loadtxt('/tmp/MHE_landmarks.txt')
poses = np.loadtxt('/tmp/MHE_outputs.txt')
# print(lms)

plt.figure()
plt.plot(lms[:,0], lms[:,1], 'kx', label='landmarks')
plt.plot(poses[:,0], poses[:,1], 'r', label='estimates')
plt.plot(x_truth, y_truth, 'b', label='truth')
plt.title('ROS Implementation')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.show()
