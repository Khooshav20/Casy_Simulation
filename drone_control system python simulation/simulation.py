from matplotlib import pyplot as plt
import numpy as np
from admittance_controller import AdmittanceController
from position_controller import PositionController
from attitude_controller import AttitudeController
from motor_mixer import MotorMixer
from drone_model import DroneModel
from config import *


def gaussian_force_applied(t, x, y, z):
    # Gaussian pulse force function
    def gaussian_pulse_force(t, amplitude, mu=1.5, sigma=0.5):
        return amplitude * np.exp(-((t - mu) ** 2) / (2 * sigma ** 2))

    F_applied = np.zeros((len(t), 3))
    F_applied[:, 0] = gaussian_pulse_force(t, x)
    F_applied[:, 1] = gaussian_pulse_force(t, y)
    F_applied[:, 2] = gaussian_pulse_force(t, z)

    return F_applied


def main():
    # Define simulation parameters
    dt = 0.01  # Time step (s)
    sim_time = 5.0  # Total simulation time

    t = np.arange(0, sim_time, dt)  # Time array
    num_steps = len(t)

    GRAPH_DATA = {
        'position': np.zeros((num_steps, 3)),
        'velocity': np.zeros((num_steps, 3)),
        'acceleration': np.zeros((num_steps, 3)),
        'desired_pitch_roll_thrust': np.zeros((num_steps, 3)),
        'actual_position': np.zeros((num_steps, 3))
    }

    F_applied = gaussian_force_applied(t, 3, 6, 9)

    admittance = AdmittanceController(M, B)
    position_ctrl = PositionController(POSITION_PID_GAINS)
    attitude_ctrl = AttitudeController(ATTITUDE_PID_GAINS)
    mixer = MotorMixer()

    drone = DroneModel([0, 0, 0])

    for i in range(1, num_steps):
        GRAPH_DATA['actual_position'][i] = drone.position

        # Admittance control
        position, velocity, acceleration = admittance.update(F_applied[i], dt)
        GRAPH_DATA['position'][i] = position
        GRAPH_DATA['velocity'][i] = velocity
        GRAPH_DATA['acceleration'][i] = acceleration
        
        # Position control
        desired_pitch, desired_roll, desired_thrust = position_ctrl.update(
            position, drone.position, dt)
        GRAPH_DATA['desired_pitch_roll_thrust'][i] = (
            desired_pitch, desired_roll, desired_thrust)

        # Attitude control
        commanded_phi, commanded_theta, commanded_psi, commanded_thrust = attitude_ctrl.update(
            desired_roll, drone.angles[0], desired_pitch, drone.angles[1], 0, drone.angles[2], desired_thrust, dt)

        # Motor mixing
        motor_omegas_sq = mixer.compute_motor_thrusts(
            commanded_thrust, commanded_phi, commanded_theta, commanded_psi)

        # Update drone model
        drone.update(motor_omegas_sq, dt)

    # Plot admittance data
    plt.figure(figsize=(10, 10))

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
        plt.plot(t, GRAPH_DATA['position'][:, j], label=f'{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m')
    plt.title('Reference Position')
    plt.grid(True)
    plt.legend(loc="upper right")

    # Plot velocity
    plt.subplot(4, 1, 3)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, GRAPH_DATA['velocity'][:, j], label=f'v{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m/s')
    plt.title('Reference Velocity')
    plt.grid(True)
    plt.legend(loc="upper right")

    # Plot acceleration
    plt.subplot(4, 1, 4)
    for j, axis in enumerate(["x", "y", "z"]):
        plt.plot(t, GRAPH_DATA['acceleration'][:, j], label=f'a{axis}r')
    plt.xlabel('Time (seconds)')
    plt.ylabel('m/s²')
    plt.title('Reference Acceleration')
    plt.grid(True)
    plt.legend(loc="upper right")

    plt.tight_layout()

    # Plot the results
    plt.figure(figsize=(10, 10))

    # plot desired thrust
    plt.subplot(3, 1, 1)
    plt.plot(t, GRAPH_DATA['actual_position']
             [:, 0], color='blue', label=f'Actual')
    plt.plot(t, GRAPH_DATA['position'][:, 0], color='red', label=f'Desired')
    plt.ylabel('x-position (m)')
    plt.xlabel('Time (seconds)')
    plt.legend(loc="upper left")
    plt.twinx()
    plt.plot(t, GRAPH_DATA['desired_pitch_roll_thrust'][:, 0], color='orange')
    plt.ylabel('Desired Pitch (rad)', color='orange')
    plt.grid(True)

    # plot desired roll
    plt.subplot(3, 1, 2)
    plt.plot(t, GRAPH_DATA['actual_position']
             [:, 1], color='blue', label=f'Actual')
    plt.plot(t, GRAPH_DATA['position']
             [:, 1], color='red', label=f'Desired')
    plt.ylabel('y-position (m)')
    plt.xlabel('Time (seconds)')
    plt.legend(loc="upper left")
    plt.twinx()
    plt.plot(t, GRAPH_DATA['desired_pitch_roll_thrust'][:, 1], color='orange')
    plt.ylabel('Desired Roll (rad)', color='orange')
    plt.grid(True)

    # plot desired thrust
    plt.subplot(3, 1, 3)
    plt.plot(t, GRAPH_DATA['actual_position']
             [:, 2], color='blue', label=f'Actual')
    plt.plot(t, GRAPH_DATA['position']
             [:, 2], color='red', label=f'Desired')
    plt.ylabel('z-position (m)')
    plt.xlabel('Time (seconds)')
    plt.legend(loc="upper left")
    plt.twinx()
    plt.plot(t, GRAPH_DATA['desired_pitch_roll_thrust'][:, 2], color='orange')
    plt.ylabel('Desired Thrust (N)', color='orange')
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
