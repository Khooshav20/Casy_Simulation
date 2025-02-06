import numpy as np
from scipy.integrate import odeint
from config import LENGTH_TO_CM, DRONE_MASS, GRAVITY, CT, CD


class DroneModel:
    def __init__(self, initial_position):
        self.position = np.array(initial_position, dtype=float)
        self.velocity = np.zeros(3)
        self.angles = np.zeros(3)  # [roll, pitch, yaw]
        self.angular_velocity = np.zeros(3)

    def update(self, motor_omegas_sq, dt):
        """
        Compute the total thrust and torques based on motor angular velocities squared.
        """
        thrusts = [CT * omega_sq for omega_sq in motor_omegas_sq.values()]
        total_thrust = sum(thrusts)

        # Torques
        tau_x = LENGTH_TO_CM * (thrusts[0] - thrusts[2])  # Roll torque
        tau_y = LENGTH_TO_CM * (thrusts[1] - thrusts[3])  # Pitch torque
        tau_z = CD * (motor_omegas_sq["omega1_sq"] - motor_omegas_sq["omega2_sq"] +
                      # Yaw torque
                      motor_omegas_sq["omega3_sq"] - motor_omegas_sq["omega4_sq"])
        total_torque = np.array([tau_x, tau_y, tau_z])

        """
        Compute the new position based on the applied force, current velocity, and acceleration.
        """
        R_B_G = self.rotation_matrix()
        gravity = np.array([0, 0, -GRAVITY])
        thrust_force = np.array([0, 0, total_thrust])  # Thrust in body frame
        acceleration = (R_B_G @ thrust_force + gravity) / DRONE_MASS

        # Update velocity and position
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        """
        Update roll (phi), pitch (theta), and yaw (psi) using continuous integration.
        """
        angular_acceleration = total_torque / \
            np.array([1.0, 1.0, 1.0])  # [rad/s^2]

        # Integrate angular velocity
        t = np.linspace(0, dt, 10)
        omega_updated = odeint(
            lambda omega, t: angular_acceleration, self.angular_velocity, t)[-1]
        self.angular_velocity = omega_updated

        # Integrate orientation
        phi, theta, psi = self.angles
        euler_to_body = np.array([
            [1, 0, -np.sin(theta)],
            [0, np.cos(phi), np.sin(phi) * np.cos(theta)],
            [0, -np.sin(phi), np.cos(phi) * np.cos(theta)]
        ])
        angular_rates = np.linalg.inv(euler_to_body) @ self.angular_velocity
        self.angles = odeint(lambda angles, t: angular_rates, [
                             phi, theta, psi], t)[-1]
        
    def get_velocity(self):
        """ Returns the current velocity of the drone. """
        return self.velocity

    def rotation_matrix(self):
        # Yaw-Pitch-Roll (ZYX) rotation matrix
        phi, theta, psi = self.angles
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])
        R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
        R_z = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi), 0],
                        [0, 0, 1]])
        return R_z @ R_y @ R_x
