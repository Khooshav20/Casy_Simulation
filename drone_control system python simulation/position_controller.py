from matplotlib import pyplot as plt
import numpy as np
from admittance_controller import gaussian_force_applied, simulate_admittance
from config import POSITION_PID_GAINS, POSITION_SATURATION_LIMIT, DRONE_MASS, GRAVITY
from pid_controller import PIDController
from drone_model import DroneModel


class PositionController:
    def __init__(self, pid_gains):
        self.pid_x = PIDController(**pid_gains['x'])
        self.pid_y = PIDController(**pid_gains['y'])
        self.pid_z = PIDController(**pid_gains['z'])

    def update(self, ref_pos, actual_pos, dt):
        print(dt, ref_pos, actual_pos)
        
        error_x = 0.7* (ref_pos[0] - actual_pos[0])     
        error_y = 0.7* (ref_pos[1] - actual_pos[1]) 
        error_z = 0.7* (ref_pos[2] - actual_pos[2]) 

        # Compute desired roll (y-axis error), pitch (x-axis error), thrust (z-axis)
        desired_pitch = self.pid_x.compute(error_x, dt)
        desired_roll = self.pid_y.compute(error_y, dt)
        desired_thrust = self.pid_z.compute(error_z, dt)

        saturated_desired_pitch = np.clip(
            desired_pitch, -POSITION_SATURATION_LIMIT['x'], POSITION_SATURATION_LIMIT['x'])
        saturated_desired_roll = np.clip(
            desired_roll, -POSITION_SATURATION_LIMIT['y'], POSITION_SATURATION_LIMIT['y'])
        saturated_desired_thrust = np.clip(
            desired_thrust, -POSITION_SATURATION_LIMIT['z'], POSITION_SATURATION_LIMIT['z'])

        return saturated_desired_pitch, saturated_desired_roll, saturated_desired_thrust


def simulate_position(desired_positions, t, dt):
    # Initialize the controller
    position_ctrl = PositionController(POSITION_PID_GAINS)
    
    desired_roll_pitch_thrust = np.zeros((len(t), 3))

    act_pos = np.zeros((len(t), 3))

    for i in range(1, len(t)):
        desired_pitch, desired_roll, desired_thrust = position_ctrl.update(
            desired_positions[i], act_pos[i], dt)
        desired_roll_pitch_thrust[i, 0] = desired_pitch
        desired_roll_pitch_thrust[i, 1] = desired_roll
        desired_roll_pitch_thrust[i, 2] = desired_thrust

        if i < len(t) - 1:
            act_pos[i + 1] = desired_positions[i]

    return desired_roll_pitch_thrust, act_pos


# Run the simulation
if __name__ == "__main__":
    # Define simulation parameters
    dt = 0.01  # Time step (s)
    sim_time = 5.0  # Total simulation time

    t = np.arange(0, sim_time, dt)

    F_applied = gaussian_force_applied(t, 3, 6, 9)

    desired_positions, _, _ = simulate_admittance(F_applied, t, dt)

    desired_roll_pitch_thrust, act_pos = simulate_position(
        desired_positions, t, dt)

    # Plot the results
    plt.figure(figsize=(12, 12))

    # plot desired thrust
    plt.subplot(4, 1, 1)
    plt.plot(t, act_pos[:, 0])
    plt.xlabel('Time (seconds)')
    plt.ylabel('Actual x-position (m)')
    plt.twinx()
    plt.plot(t, desired_roll_pitch_thrust[:, 0], color='orange')
    plt.ylabel('Desired Pitch (rad)')
    plt.grid(True)

    # plot desired roll
    plt.subplot(4, 1, 2)
    plt.plot(t, act_pos[:, 1])
    plt.ylabel('Actual y-position (m)')
    plt.xlabel('Time (seconds)')
    plt.twinx()
    plt.plot(t, desired_roll_pitch_thrust[:, 1], color='orange')
    plt.ylabel('Desired Roll (rad)')
    plt.grid(True)

    # plot desired thrust
    plt.subplot(4, 1, 3)
    plt.plot(t, act_pos[:, 2])
    plt.ylabel('Actual z-position (m)')
    plt.xlabel('Time (seconds)')
    plt.twinx()
    plt.plot(t, desired_roll_pitch_thrust[:, 2], color='orange')
    plt.ylabel('Desired Thrust (N)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()
