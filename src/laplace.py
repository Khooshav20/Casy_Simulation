import numpy as np
from scipy.integrate import odeint

# Prompt user for the force magnitudes in x, y, z directions
force_x = float(input("Enter the force magnitude in the x direction (N): "))
force_y = float(input("Enter the force magnitude in the y direction (N): "))
force_z = float(input("Enter the force magnitude in the z direction (N): "))

# Define constants
B = 5  # Damping coefficient
m = 2  # Mass
mu = 1.5  # Mean of Gaussian force
sigma = 0.5  # Standard deviation of Gaussian force
t_value = 10  # Time to evaluate the results

# Gaussian force as a function of time for each axis
def gaussian_force(t, force_magnitude):
    return force_magnitude * np.exp(-((t - mu)**2) / (2 * sigma**2))

# Define the system of equations: dv/dt = acceleration, dx/dt = velocity
def system(state, t, force_magnitude):
    x, v = state  # State contains position x and velocity v
    F = gaussian_force(t, force_magnitude)  # Force at time t
    a = (F - B * v) / m  # Acceleration
    return [v, a]  # dx/dt = v, dv/dt = a

# Time array
time = np.linspace(0, t_value, 1000)

# Initial conditions: x(0) = 0, v(0) = 0
initial_state = [0, 0]

# Solve the ODEs for x direction
solution_x = odeint(system, initial_state, time, args=(force_x,))
position_x = solution_x[:, 0]
velocity_x = solution_x[:, 1]
position_x_at_t = position_x[-1]
velocity_x_at_t = velocity_x[-1]

# Solve the ODEs for y direction
solution_y = odeint(system, initial_state, time, args=(force_y,))
position_y = solution_y[:, 0]
velocity_y = solution_y[:, 1]
position_y_at_t = position_y[-1]
velocity_y_at_t = velocity_y[-1]

# Solve the ODEs for z direction
solution_z = odeint(system, initial_state, time, args=(force_z,))
position_z = solution_z[:, 0]
velocity_z = solution_z[:, 1]
position_z_at_t = position_z[-1]
velocity_z_at_t = velocity_z[-1]

# Print results
print(f"At t = {t_value} seconds:")
print(f"X Direction -> Velocity: {velocity_x_at_t}, Position: {position_x_at_t}")
print(f"Y Direction -> Velocity: {velocity_y_at_t}, Position: {position_y_at_t}")
print(f"Z Direction -> Velocity: {velocity_z_at_t}, Position: {position_z_at_t}")