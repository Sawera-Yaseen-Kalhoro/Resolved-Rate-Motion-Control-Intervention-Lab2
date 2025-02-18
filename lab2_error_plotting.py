import numpy as np
import matplotlib.pyplot as plt

# Load the error evolution from all 3 methods
error_transpose = np.load("Error_transpose.npy")
error_pseudoinverse = np.load("Error_pseudoinverse.npy")
error_DLS = np.load("Error_DLS.npy")

# Simulation parameters
dt = 1.0/60.0 # sampling time
tt = np.arange(0, 10, dt) # time vector

# Plot the error evolution
fig = plt.figure()
ax = fig.add_subplot(111, xlim=(0, 10))
ax.set_title('Resolved-rate Motion Control - Error Evolution')
ax.set_xlabel('Time[s]')
ax.set_ylabel('Error[m]')
# ax.set_aspect('equal')
ax.grid()

ax.plot(tt,error_transpose[:tt.shape[0]],
         tt,error_pseudoinverse[:tt.shape[0]],
         tt,error_DLS[:tt.shape[0]])
ax.legend(['Transpose','Pseudoinverse','DLS'])

plt.show()
