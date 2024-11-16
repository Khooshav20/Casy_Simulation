#!/usr/bin/env python

import rospy
from casy_simulation.srv import StartSimulation, StartSimulationResponse

class SimulationServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('simulation_server')
        
        # Define the 'start_simulation' service
        self.start_simulation_service = rospy.Service('/start_simulation', StartSimulation, self.handle_start_simulation)

    def handle_start_simulation(self, req):
        """
        Handle the start simulation request by returning success = True
        """
        rospy.loginfo("Simulation start request received.")
        # Simulate starting the simulation by responding with success = True
        return StartSimulationResponse(success=True)

    def run(self):
        # Keep the service node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Instantiate and run the simulation server
        server = SimulationServer()
        server.run()
    except rospy.ROSInterruptException:
        pass