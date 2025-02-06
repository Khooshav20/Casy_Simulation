class PIDController:
    def __init__(self, kp, ki, kd, N):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.N = N    # Filter coefficient for derivative term

        # States
        self.integral = 0.0  # Integral term
        self.prev_error = 0.0  # Previous error for derivative calculation
        self.filtered_derivative_prev = 0.0  # Previous filtered derivative value

    def compute(self, error, dt):
        if dt <= 0:
            raise ValueError("Time step (dt) must be positive.")

        # Proportional term
        P = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        I = self.ki * self.integral

        # Derivative term with low-pass filter
        delta_error = error - self.prev_error
        # Filtered derivative: (N * delta_error + prev_filtered) / (1 + N * dt)
        filtered_derivative = (
            self.N * delta_error + self.filtered_derivative_prev
        ) / (1 + self.N * dt)
        D = self.kd * filtered_derivative

        # Update states
        self.filtered_derivative_prev = filtered_derivative
        self.prev_error = error

        # Return control output
        return P + I + D


def saturate(value, limit):
    return max(-limit, min(limit, value))
