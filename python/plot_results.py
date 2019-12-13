import numpy as np
import matplotlib.pyplot as plt

lms = np.loadtxt('/tmp/MHE_landmarks.txt')
poses = np.loadtxt('/tmp/MHE_outputs.txt')

plt.figure()
plt.plot(lms[:,0], lms[:,1], 'kx', label='landmarks')
plt.plot(poses[:,0], poses[:,1], 'b', label='estimates')
plt.legend()
plt.show()
