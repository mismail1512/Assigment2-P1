#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point  # To match the target data type

class TargetService:
    def __init__(self):
        # Holds the last target that was set
        self.last_target = None

        # Initialize service for retrieving last target information
        rospy.Service('/retrieve_last_target', Trigger, self.process_request)

    def process_request(self, request):
        """Handles the service request for the last target."""
        if self.last_target:
            message = f"Last target: x={self.last_target.x}, y={self.last_target.y}"
            return TriggerResponse(success=True, message=message)
        else:
            return TriggerResponse(success=False, message="No target has been set.")

    def update_last_target(self, x, y):
        """Updates the last target."""
        self.last_target = Point(x=x, y=y)
        rospy.loginfo(f"Last target updated: x={x}, y={y}")

if __name__ == "__main__":
    rospy.init_node('target_service_node')

    # Initialize the service node
    service_node = TargetService()

    # Simulating target updates for testing
    # In practice, this would be updated based on real targets from the action client
    service_node.update_last_target(1.0, 2.0)  # Example of setting a target

    # Keep the service node alive
    rospy.spin()

