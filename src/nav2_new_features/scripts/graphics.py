import numpy as np
import matplotlib.pyplot as plt
import pdb
import transformations as tf
data = np.loadtxt('poses.txt')
times = np.loadtxt('times.txt')
transformations = data.reshape((-1, 3, 4))
positions = transformations[:, :, 3]

plt.plot(positions[:,0], positions[:,1], "r+")
plt.show()
orientations = transformations[:, :3, :3]  # You might need to convert these to Euler angles or quaternions
linear_velocities = np.diff(positions, axis=0)
angular_velocities = np.diff(orientations, axis=0)  # This might not give you what you want if the orientations are not represented as Euler angles
dt = np.diff(times)
dt = dt.reshape((74, 1))
positions = np.cumsum(linear_velocities * dt, axis=0)
dt = dt[:, np.newaxis]      
orientations = np.cumsum(angular_velocities * dt, axis=0)
#convert orientations to Euler angles
euler_orientations = np.empty((orientations.shape[0], 3))
for i in range(orientations.shape[0]):
	T = np.eye(4)
	T[:3, :3] = orientations[i]
	euler_orientations[i] = tf.euler_from_matrix(T, 'sxyz')
print(euler_orientations.shape)
