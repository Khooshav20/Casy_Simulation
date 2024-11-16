import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Prompt user for the force magnitude
force_magnitude = float(input("Enter the force magnitude (N): "))

# Define constants
B = 5  # Damping coefficient
m = 2  # Mass
mu = 1.5  # Mean of Gaussian force
sigma = 0.5  # Standard deviation of Gaussian force
t_value = 10  # Time to evaluate the results

# Gaussian force as a function of time
def gaussian_force(t):
    return force_magnitude * np.exp(-((t - mu)**2) / (2 * sigma**2))

# Define the system of equations: dv/dt = acceleration, dx/dt = velocity
def system(state, t):
    x, v = state  # State contains position x and velocity v
    F = gaussian_force(t)  # Force at time t
    a = (F - B * v) / m  # Acceleration
    return [v, a]  # dx/dt = v, dv/dt = a

# Time array
time = np.linspace(0, t_value, 1000)

# Initial conditions: x(0) = 0, v(0) = 0
initial_state = [0, 0]

# Solve the ODEs
solution = odeint(system, initial_state, time)
position = solution[:, 0]
velocity = solution[:, 1]

# Evaluate position and velocity at t = t_value
position_at_t = position[-1]
velocity_at_t = velocity[-1]

# Print results
print(f"At t = {t_value} seconds:")
print(f"Velocity, v(t): {velocity_at_t}")
print(f"Position, x(t): {position_at_t}")