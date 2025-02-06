# Physical parameters
M = 2.0          # Virtual mass (kg)
B = 5.0          # Damping coefficient (Ns/m)
GRAVITY = 9.81   # m/s²

# PID Gains
POSITION_PID_GAINS = {
    "x": {"kp": 1.794, "kd": 0.222, "ki": 2.286, "N": 1996.59},
    "y": {"kp": -1.6514, "kd": -0.531, "ki": -0.832, "N": 2968.47},
    "z": {"kp": 17.74, "kd": 0.220, "ki": 49.219, "N": 29.427}
}
ATTITUDE_PID_GAINS = {
    "phi": {"kp": 8.701, "ki": 0.776, "kd": 14.8, "N": 572.77},
    "theta": {"kp": 27.687, "ki": 7.524, "kd": 23.422, "N": 907.78},
    "psi": {"kp": 27.687, "ki": 7.524, "kd": 23.422, "N": 907.78},
}

POSITION_SATURATION_LIMIT = {
    "x": 0.3927,   # ±π/8 radians (22.5 degrees)
    "y": 0.3927,   # ±π/8 radians (22.5 degrees)
    "z": 12
}

# Drone parameters
DRONE_MASS = 1.347  # [kg] Mass of the drone including batteries
LENGTH_TO_CM = 0.2405  # [m] Length from CM to propellers in any 4 directions
CT = 0.35  # Thrust coefficient of propellers
CD = 0.65  # Drag coefficient
