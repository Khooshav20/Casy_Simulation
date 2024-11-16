#!/usr/bin/env python
from datetime import datetime
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
import sys
import threading
import tf

class CASyController:
    def __init__(self):
        print("Initializing ROS node...")
        rospy.init_node('casy_controller', anonymous=True)
        print("ROS node initialized")

        # System constants
        self.g = 9.81  # [m/s^2] Acceleration due to gravity
        self.m = 1.347  # [kg] Mass of the drones including batteries
        self.l = 0.2405  # [m] Length from CM to propellers in any 4 directions (not implemented currently)
        self.CT = 0.35  # Thrust coefficient of propellers (not implemented currently)
        self.CD = 0.65  # Drag coefficient (not implemented currently)
        self.B =10  # Damping coefficient 

        # Initial states
        self.position = np.array([0.5, 0.0, 0.0])  # Starting position (x, y, z)
        self.velocity = np.array([0.0, 0.0, 0.0])  # Initial velocity (vx, vy, vz)
        self.acceleration = np.array([0.0, 0.0, 0.0])  # Initial acceleration (ax, ay, az)
        self.dt = 0.05  # Slightly larger time step for visible updates

        # Initial orientation (yaw, pitch, roll) and quaternion
        self.yaw = np.radians(30)    # The left and right rotation of the drone, along the z-axis- 30 degrees yaw
        self.pitch = np.radians(15)  # The forward and backward tilt of the drone, along the y-axis- 15 degrees pitch
        self.roll = np.radians(45)   # The side-to-side tilt of the drone, along the x-axis- 45 degrees roll
        self.orientation = np.array([self.roll, self.pitch, self.yaw])  # roll, pitch, yaw
        self.quaternion = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # Publishers
        self.applied_force_pub = rospy.Publisher('/applied_force', Float32MultiArray, queue_size=10)
        self.velocity_pub = rospy.Publisher('/drone_velocity', Vector3, queue_size=10)
        self.orientation_pub = rospy.Publisher('/drone_orientation', Quaternion, queue_size=10)

        # Exit flag
        self.exit_flag = False

        print("Initialization complete")

    def compute_position(self, applied_force):
        """
        Compute the new position based on the applied force, current velocity, and acceleration.
        """
        # Calculate acceleration (F = ma)
        self.acceleration = applied_force / self.m
        
        # Update velocity (v = u + at) and apply damping
        self.velocity += self.acceleration * self.dt
        self.velocity -= self.B * self.velocity * self.dt  # Apply damping

        # Update position (s = s0 + vt)
        self.position += self.velocity * self.dt

    def update_orientation(self):
        """
        Update orientation using a predefined quaternion for simulation purposes.
        """
        # Convert quaternion to yaw, pitch, roll
        self.orientation = tf.transformations.euler_from_quaternion(self.quaternion)

    def display_status(self):
        """
        Display current position, velocity, orientation (yaw, pitch, roll), and timestamp in a table format.
        """
        # Get the current time in a readable format (HH:MM:SS)
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Update orientation (simulated quaternion to Euler angles)
        self.update_orientation()

        # Print headers if this is the first time displaying status
        if not hasattr(self, 'header_printed'):
            print(f"{'Time':<10} {'Position (x)':<15} {'Position (y)':<15} {'Position (z)':<15} "
                f"{'Velocity (vx)':<15} {'Velocity (vy)':<15} {'Velocity (vz)':<15} "
                f"{'Yaw':<10} {'Pitch':<10} {'Roll':<10}")
            print("=" * 135)  # Header separator line
            self.header_printed = True

        # Print values in table format
        print(f"{timestamp:<10} {self.position[0]:<15.2f} {self.position[1]:<15.2f} {self.position[2]:<15.2f} "
            f"{self.velocity[0]:<15.2f} {self.velocity[1]:<15.2f} {self.velocity[2]:<15.2f} "
            f"{np.degrees(self.orientation[0]):<10.2f} {np.degrees(self.orientation[1]):<10.2f} {np.degrees(self.orientation[2]):<10.2f}")
        
       
        print("-" * 135)

        # Flush output to prevent rows from being combined in the terminal buffer
        sys.stdout.flush()

    def check_for_exit(self):
        """
        Check for Enter key press to exit.
        """
        input("Press Enter at any time to exit...\n\n")
        self.exit_flag = True

    def run(self):
        """
        Main function to run the simulation based on user input.
        """
        # Prompt the user to start the simulation
        run_simulation = input("Do you want to run the simulation? (y/n): ").strip().lower()
        if run_simulation == 'y':
            try:
                # Prompt the user for applied force values
                applied_force_input = input("Enter applied force values as x,y,z (e.g., 0,15,5): ").strip()
                applied_force_values = [float(value) for value in applied_force_input.split(",")]
                
                if len(applied_force_values) == 3:
                    print("Applied force values set to:", applied_force_values)

                    # Publish the applied force values for logging or debugging
                    applied_force_array = Float32MultiArray(data=applied_force_values)
                    self.applied_force_pub.publish(applied_force_array)
                    rospy.loginfo("Applied force published: {}".format(applied_force_values))

                    # Convert the applied force to a NumPy array for calculations
                    applied_force = np.array(applied_force_values)

                    # Start the thread to check for "Enter" key press
                    exit_thread = threading.Thread(target=self.check_for_exit)
                    exit_thread.start()

                    # Run the main loop
                    rate = rospy.Rate(2)  # Adjusted rate for slower output (0.5 Hz for slower updates)
                    while not rospy.is_shutdown() and not self.exit_flag:
                        # Update position
                        self.compute_position(applied_force)

                        # Publish updated velocity
                        velocity_msg = Vector3(*self.velocity)
                        self.velocity_pub.publish(velocity_msg)

                        # Publish the simulated orientation as quaternion
                        orientation_msg = Quaternion(*self.quaternion)
                        self.orientation_pub.publish(orientation_msg)

                        # Display status in single line with timestamp
                        self.display_status()

                        rate.sleep()
                    
                    print("\nExiting simulation.")
                else:
                    rospy.loginfo("Error: Applied force matrix is not in the correct format. Please enter three values for x, y, z.")
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
