import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# MPC parameters
N = 10              # Prediction horizon
T = 0.1             # Sampling time
d_safe = 10         # Safe following distance
qv = 1              # Velocity error weight
r = 0.1             # Acceleration penalty weight
amin, amax = -3, 2  # Acceleration limits

# Leader trajectory (position and velocity)
def generate_leader_trajectory(total_steps):
    x = 0
    v = 15
    traj = []
    for _ in range(total_steps):
        traj.append([x, v])
        x += v * T
    return np.array(traj)

# Cost function
def cost(u, x0, leader_traj):
    x = x0.copy()
    J = 0
    for k in range(N):
        a = u[k]
        x[0] += x[1] * T
        x[1] += a * T

        x1_k, v1_k = leader_traj[k]
        pos_err = (x1_k - x[0] - d_safe) ** 2
        vel_err = qv * (v1_k - x[1]) ** 2
        acc_cost = r * (a ** 2)

        J += pos_err + vel_err + acc_cost
    return J

# MPC controller
def mpc_control(x0, leader_traj):
    u0 = np.zeros(N)
    bounds = [(amin, amax)] * N
    res = minimize(cost, u0, args=(x0, leader_traj), bounds=bounds, method='SLSQP')
    return res.x[0]  # apply first control input

# Simulate
total_steps = 100
x2 = 0
v2 = 10
states = []
leader_traj_full = generate_leader_trajectory(total_steps + N)

for t in range(total_steps):
    x0 = [x2, v2]
    leader_pred = leader_traj_full[t:t+N]
    a2 = mpc_control(x0, leader_pred)
    x2 += v2 * T
    v2 += a2 * T
    states.append([x2, v2])

# Plot
leader_pos = leader_traj_full[:total_steps, 0]
follower_pos = [s[0] for s in states]

plt.plot(leader_pos, label="Leader")
plt.plot(follower_pos, label="Follower")
plt.xlabel("Time step")
plt.ylabel("Position (m)")
plt.legend()
plt.title("MPC 2-Car Following")
plt.grid(True)
plt.show()
