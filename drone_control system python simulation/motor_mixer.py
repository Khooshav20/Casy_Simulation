import numpy as np

from pid_controller import saturate


class MotorMixer:
    def compute_motor_thrusts(self, commanded_thrust, commanded_phi, commanded_theta, commanded_psi):
        # Apply saturations
        saturated_commanded_psi = saturate(commanded_psi, 1e4)
        saturated_commanded_phi = saturate(commanded_phi, 1e4)
        saturated_commanded_theta = saturate(commanded_theta, 1e4)
        saturated_commanded_thrust = saturate(commanded_thrust, 1e4)

        # Compute motor squared angular velocities
        omega1_sq = saturated_commanded_thrust - saturated_commanded_theta + \
            saturated_commanded_phi - saturated_commanded_psi
        omega2_sq = saturated_commanded_thrust + saturated_commanded_theta - \
            saturated_commanded_phi - saturated_commanded_psi
        omega3_sq = saturated_commanded_thrust + saturated_commanded_theta + \
            saturated_commanded_phi + saturated_commanded_psi
        omega4_sq = saturated_commanded_thrust - saturated_commanded_theta - \
            saturated_commanded_phi + saturated_commanded_psi

        # Ensure non-negative motor outputs
        omega1_sq = max(omega1_sq, 0)
        omega2_sq = max(omega2_sq, 0)
        omega3_sq = max(omega3_sq, 0)
        omega4_sq = max(omega4_sq, 0)

        # Return motor outputs
        return {
            "omega1_sq": omega1_sq,
            "omega2_sq": omega2_sq,
            "omega3_sq": omega3_sq,
            "omega4_sq": omega4_sq
        }
