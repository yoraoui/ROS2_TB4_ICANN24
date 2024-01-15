import numpy as np
#read velovity text file in python


velocities = np.loadtxt('velocities.txt')
input_vels = np.empty((velocities.shape[0], 3))
for i in range(velocities.shape[0]):    
    row = velocities[i]
    input_vels[i] = np.array([row[0], np.sin(row[1]), np.cos(row[1])])

np.savetxt('inputs_vels.txt', input_vels, fmt='%.6f')