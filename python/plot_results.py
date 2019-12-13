import numpy as np
import matplotlib.pyplot as plt
from extractdata import *

lms = np.loadtxt('/tmp/MHE_landmarks.txt')
poses = np.loadtxt('/tmp/MHE_outputs.txt')

plt.figure()
plt.plot(lms[:,1], lms[:,0], 'kx', label='landmarks')
plt.plot(poses[:,1], poses[:,0], 'b', label='estimates')
plt.plot(x_truth, y_truth, 'g', label='truth')
plt.legend()
plt.show()
