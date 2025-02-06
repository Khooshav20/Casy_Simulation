
from matplotlib import pyplot as plt
import numpy as np


class AdmittanceController:
    def __init__(self, M, B):
        self.M = M
        self.B = B
        self.velocity = 0.0
        self.position = 0.0

    def update(self, F_applied, dt):
        # Eq.2.2: acceleration = (F_applied - B*velocity) / M
        acceleration = (F_applied - self.B * self.velocity) / self.M

        # Integrate acceleration to velocity
        self.velocity += acceleration * dt

        # Integrate velocity to position
        self.position += self.velocity * dt

        return self.position, self.velocity, acceleration


def gaussian_force_applied(t, x, y, z):
    # Gaussian pulse force function
    def gaussian_pulse_force(t, amplitude, mu=1.5, sigma=0.5):
        return amplitude * np.exp(-((t - mu) ** 2) / (2 * sigma ** 2))

    F_applied = np.zeros((len(t), 3))
    F_applied[:, 0] = gaussian_pulse_force(t, x)
    F_applied[:, 1] = gaussian_pulse_force(t, y)
    F_applied[:, 2] = gaussian_pulse_force(t, z)

    return F_applied


def simulate_admittance(F_applied, t, dt):
    # Define system parameters
    M = 2.0  # Mass (kg)
    B = 5  # Damping coefficient (Ns/m)

    # Initialize the controller
    admittance = AdmittanceController(M, B)

    num_steps = len(t)

    # Solve the system for each force
    positions = np.zeros((num_steps, 3))
    velocities = np.zeros((num_steps, 3))
    accelerations = np.zeros((num_steps, 3))

    for i in range(1, num_steps):
        position, velocity, acceleration = admittance.update(F_applied[i], dt)
        positions[i] = position
        velocities[i] = velocity
        accelerations[i] = acceleration

    return positions, velocities, accelerations


# Run the simulation
if __name__ == "__main__":
    # Define simulation parameters
    dt = 0.01  # Time step (s)
    sim_time = 5.0  # Total simulation time

    t = np.arange(0, sim_time, dt)

    F_applied = gaussian_force_applied(t, 3, 6, 9)

    positions, velocities, accelerations = simulate_admittance(
        F_applied, t, dt)

    # Plot the results
    plt.figure(figsize=(12, 12))

    # Plot applied force
    plt.subplot(4, 1, 1)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, F_applied[:, j], label=f'F{axis}')
    plt.xlabel('Time (seconds)')
    plt.ylabel('N')
    plt.title('Applied Force')
    plt.grid(True)
    plt.legend(loc="upper right")

    # Plot position
    plt.subplot(4, 1, 2)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, positions[:, j], label=f'{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m')
    plt.title('Reference Position')
    plt.grid(True)
    plt.legend(loc="upper right")

    # Plot velocity
    plt.subplot(4, 1, 3)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, velocities[:, j], label=f'v{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m/s')
    plt.title('Reference Velocity')
    plt.grid(True)
    plt.legend(loc="upper right")

    # Plot acceleration
    plt.subplot(4, 1, 4)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, accelerations[:, j], label=f'a{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m/s²')
    plt.title('Reference Acceleration')
    plt.grid(True)
    plt.legend(loc="upper right")

    plt.tight_layout()
    plt.show()
