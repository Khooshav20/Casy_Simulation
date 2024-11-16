# Minimum CMake version required
cmake_minimum_required(VERSION 3.0.2)
project(casy_simulation)

# Find required catkin components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Add service files located in the srv directory
add_service_files(
  FILES
  StartSimulation.srv
)

# Generate messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare the catkin package with message_runtime to allow other packages to use generated messages/services
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Specify locations of Python scripts
catkin_install_python(PROGRAMS
  src/casy_controller.py
  src/simulation_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install any additional launch files if needed (optional)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)