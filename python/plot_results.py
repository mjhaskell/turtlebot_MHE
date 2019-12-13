import numpy as np
import matplotlib.pyplot as plt

lms = np.loadtxt('/tmp/MHE_landmarks.txt')
poses = np.loadtxt('/tmp/MHE_outputs.txt')
print(lms)

plt.figure()
plt.plot(lms[:,1], lms[:,0], 'kx', label='landmarks')
plt.plot(poses[:,0], poses[:,1], 'b', label='estimates')
plt.legend()
plt.show()
