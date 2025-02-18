# Import necessary libraries
from lab2_robotics import * # Import our library (includes Numpy)
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition (planar 2 link manipulator)
d = np.zeros(2)           # displacement along Z-axis
q = np.array([0.2, 0.5])  # rotation around Z-axis (theta)
a = np.array([0.75, 0.5]) # displacement along X-axis
alpha = np.zeros(2)       # rotation around X-axis 

# Simulation params
dt = 0.01 # Sampling time
Tt = 10 # Total simulation time
tt = np.arange(0, Tt, dt) # Simulation time vector

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Kinematics')
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'r-', lw=1) # End-effector path

# Memory
PPx = []
PPy = []
q1 = [q[0]] # initialize memory of joint position q1
q2 = [q[1]] # initialize memory of joint position q2

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    return line, path

# Simulation loop
def simulate(t):
    global d, q, a, alpha
    global PPx, PPy
    global q1, q2
    
    # Update robot
    T = kinematics(d, q, a, alpha)
    dq = np.array([0.5,1.0]) # Define how joint velocity changes with time!
    q = q + dt * dq
    
    # Update drawing
    PP = robotPoints2D(T)
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)

    # Store the joint positions
    q1.append(q[0])
    q2.append(q[1])
    
    return line, path

# Function to generate the second plot
def joint_pos_plot():
    # Create a second plot for the joint positions
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, xlim=(0, Tt))
    ax2.set_title('Joint Positions')
    ax2.set_xlabel('Time[s]')
    ax2.set_ylabel('Angle[rad]')
    # ax2.set_aspect('equal')
    ax2.grid()

    ax2.plot(tt,q1[:tt.shape[0]], tt,q2[:tt.shape[0]])
    ax2.legend(['q1','q2'])
    plt.show()

# Run simulation
animation = anim.FuncAnimation(fig, simulate, tt, 
                                interval=10, blit=True, init_func=init, repeat=False)
plt.show()

# Show the plot of joint positions
joint_pos_plot()