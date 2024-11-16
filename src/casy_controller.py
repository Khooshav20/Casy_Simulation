#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
from casy_simulation.srv import StartSimulation, StartSimulationResponse
import numpy as np

class CASyController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('casy_controller', anonymous=True)

        # System constants
        self.g = 9.81  # [m/s^2] Acceleration due to gravity
        self.m = 1.347  # [kg] Mass of the drones including batteries
        self.l = 0.2405  # [m] Length from CM to propellers in any 4 directions
        self.CT = 0.35  # Thrust coefficient of propellers
        self.CD = 0.65  # Drag coefficient
        self.B = 10  # Damping coefficient

        # Initial states
        self.position = np.array([0.5, 0.0, 0.0])  # Starting position (x, y, z)
        self.velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity (vx, vy, vz)
        self.acceleration = np.array([0.0, 0.0, 0.0])  # Initial acceleration (ax, ay, az)
        self.dt = 0.01  # Time step for simulation update (e.g., 0.01s for 100Hz updates)

        # Publisher for applied force
        self.force_pub = rospy.Publisher('/applied_force', Float32MultiArray, queue_size=10)

        # Position publisher (optional)
        self.position_pub = rospy.Publisher('/drone_position', Vector3, queue_size=10)
        

        # Service to start the simulation
        rospy.wait_for_service('/start_simulation')
        self.start_simulation_service = rospy.ServiceProxy('/start_simulation', StartSimulation)

    def compute_position(self, force):
        """
        Compute the new position based on the applied force, current velocity, and acceleration.
        """
        self.acceleration = force / self.m  # Calculate acceleration (F = ma)
        self.velocity += self.acceleration * self.dt  # Update velocity
        self.velocity -= self.B * self.velocity * self.dt  # Apply damping
        self.position += self.velocity * self.dt  # Update position

    def run(self):
        """
        Main function to run the simulation based on user input.
        """
        run_simulation = input("Do you want to run the simulation? (y/n): ").strip().lower()
        if run_simulation == 'y':
            try:
                force_input = input("What is the applied force? (write it as a 1x3 list, e.g., [0, 0, 0]): ")
                force_values = eval(force_input)  # Convert input string to list

                if isinstance(force_values, list) and len(force_values) == 3:
                    force_array = Float32MultiArray(data=force_values)
                    self.force_pub.publish(force_array)
                    rospy.loginfo("Applied force published: {}".format(force_values))

                    force = np.array(force_values)
                    response = self.start_simulation_service()
                    if response.success:
                        rospy.loginfo("Simulation started successfully.")
                        rate = rospy.Rate(100)  # Set rate to 100Hz
                        while not rospy.is_shutdown():
                            self.compute_position(force)  # Update position
                            rospy.loginfo(f"Updated position: {self.position}")
                            self.position_pub.publish(Vector3(*self.position))
                            rate.sleep()
                    else:
                        rospy.loginfo("Failed to start the simulation.")
                else:
                    rospy.loginfo("Error: Force matrix is not in the correct format.")
            except Exception as e:
                rospy.logerr("An error occurred: {}".format(e))
        else:
            rospy.loginfo("Ending session.")

if __name__ == '__main__':
    try:
        controller = CASyController()
        controller.run()
    except rospy.ROSInterruptException:
        pass