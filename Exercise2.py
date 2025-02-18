# Import necessary libraries
from lab2_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition
d = np.zeros(2)           # displacement along Z-axis
q = np.array([0.2, 0.5])  # rotation around Z-axis (theta)
a = np.array([0.75, 0.5]) # displacement along X-axis
alpha = np.zeros(2)       # rotation around X-axis 
revolute = [True, True]
sigma_d = np.array([0.0, 1.0])
K = np.diag([1, 1])

# Simulation params
dt = 1.0/60.0

# Controller method
controller = 'DLS' # options: 'transpose', 'pseudoinverse', or 'DLS'
label = "" # initialize a string variable to name the file storing the error evolution

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'c-', lw=1) # End-effector path
point, = ax.plot([], [], 'rx') # Target
PPx = []
PPy = []

error_evolution = [] # initialize a memory to store the error evolution

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    return line, path, point

# Simulation loop
def simulate(t):
    global d, q, a, alpha, revolute, sigma_d
    global PPx, PPy
    global error_evolution
    global controller, label

    # Update robot
    T = kinematics(d, q, a, alpha)
    J = jacobian(T, revolute) # Implement!

    # Update control
    sigma = T[-1][:2,-1]      # Position of the end-effector
    err = sigma_d - sigma          # Control error
    error_evolution.append(np.linalg.norm(err)) # store the error evolution

    # Control solution
    if controller == 'transpose': # transpose method
        dq = (J[:2,:].T @ np.array([(K@err)]).T).reshape((2,)) 
        label = "Error_transpose"
    elif controller == 'pseudoinverse': # pseudoinverse method
        dq = (np.linalg.pinv(J[:2,:]) @ np.array([(K@err)]).T).reshape((2,))
        label = "Error_pseudoinverse"
    elif controller == 'DLS': # DLS method
        dq = (DLS(J[:2,:],0.1) @ np.array([(K@err)]).T).reshape((2,))
        label = "Error_DLS"

    q += dt * dq
    
    # Update drawing
    P = robotPoints2D(T)
    line.set_data(P[0,:], P[1,:])
    PPx.append(P[0,-1])
    PPy.append(P[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_d[0], sigma_d[1])

    return line, path, point

# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=False)
plt.show()

# Save the error evolution for future plotting
np.save(label,error_evolution)