import numpy as np

from config import GRAVITY
from pid_controller import PIDController


class AttitudeController:
    def __init__(self, pid_gains):
        self.pid_phi = PIDController(**pid_gains['phi'])
        self.pid_theta = PIDController(**pid_gains['theta'])
        self.pid_psi = PIDController(**pid_gains['psi'])

    def update(self, desired_phi, actual_phi, desired_theta, actual_theta, desired_psi, actual_psi, desired_thrust, dt):
        error_phi = desired_phi - actual_phi
        error_theta = desired_theta - actual_theta
        error_psi = desired_psi - actual_psi

        commanded_phi = self.pid_phi.compute(error_phi, dt)
        commanded_theta = self.pid_theta.compute(error_theta, dt)
        commanded_psi = self.pid_psi.compute(error_psi, dt)
        commanded_thrust = desired_thrust + (1.347 * GRAVITY / (0.35 * 4))

        return commanded_phi, commanded_theta, commanded_psi, commanded_thrust
