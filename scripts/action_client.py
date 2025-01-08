#!/usr/bin/env python3

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotState  # Custom message for robot's state
from geometry_msgs.msg import Point  # For sending the last target

class GoalManager:
    def __init__(self):
        rospy.init_node('goal_manager', anonymous=True)

        # Initialize the connection to the action server
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Connecting to action server...")
        self.action_client.wait_for_server()
        rospy.loginfo("Action server connected successfully.")

        # Publishers to share robot state and target updates
        self.robot_status_publisher = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        self.target_publisher = rospy.Publisher('/last_target', Point, queue_size=10)

        # Subscriber to monitor odometry and update robot status
        rospy.Subscriber('/odom', Odometry, self.update_robot_status)

        # Tracks the robot's current state
        self.current_state = RobotState()
        self.current_target = Point()

        # To check if goal is still active
        self.goal_active = False

    def update_robot_status(self, odom_data):
        """Processes odometry data to update the robot's status."""
        self.current_state.x = odom_data.pose.pose.position.x
        self.current_state.y = odom_data.pose.pose.position.y
        self.current_state.vel_x = odom_data.twist.twist.linear.x
        self.current_state.vel_z = odom_data.twist.twist.angular.z

        # Publish the robot's updated state if the goal is active
        if self.goal_active:
            try:
                self.robot_status_publisher.publish(self.current_state)
            except rospy.ROSException:
                rospy.logwarn("Attempted to publish to a closed topic.")

    def set_target(self, x, y):
        """Sends a target location to the action server."""
        new_goal = PlanningGoal()
        new_goal.target_pose.pose.position.x = x
        new_goal.target_pose.pose.position.y = y

        # Save and share the target point
        self.current_target.x = x
        self.current_target.y = y
        self.target_publisher.publish(self.current_target)

        # Dispatch the goal to the action server
        self.action_client.send_goal(new_goal, feedback_cb=self.handle_feedback)
        rospy.loginfo(f"Target dispatched: x={x}, y={y}")

        # Set goal as active
        self.goal_active = True

    def handle_feedback(self, feedback):
        """Logs feedback received from the action server."""
        rospy.loginfo(f"Feedback from server: {feedback}")

    def stop_target(self):
        """Cancels the currently active target."""
        self.action_client.cancel_goal()
        self.goal_active = False  # Set goal to inactive after cancellation
        rospy.loginfo("Target operation canceled.")

    def execute(self):
        """Main interactive loop for managing targets."""
        rospy.loginfo("Goal Manager is operational.")
        while not rospy.is_shutdown():
            while self.goal_active:
                print("\nAvailable Commands:")
                print("  [2] Cancel the active target")
                print("  [3] Quit")
                user_input = input("Enter your choice: ").strip().lower()

                if user_input == '2':
                    self.stop_target()

                    # After canceling, ask the user if they want to set another goal
                    continue_input = input("Do you want to set another target? (yes/no): ").strip().lower()
                    if continue_input != 'yes':
                        rospy.loginfo("Exiting the goal manager.")
                        break
                    else:
                        # If they want to set another target, break the inner loop and ask for new target
                        continue
                elif user_input == '3':
                    rospy.loginfo("Exiting the goal manager.")
                    break

            if not self.goal_active:
                print("\nAvailable Commands:")
                print("  [1] Set a new target")
                print("  [2] Cancel the active target (if any)")
                print("  [3] Quit")
                user_input = input("Enter your choice: ").strip().lower()

                if user_input == '1':
                    try:
                        target_x = float(input("Enter x-coordinate: "))
                        target_y = float(input("Enter y-coordinate: "))
                        self.set_target(target_x, target_y)
                    except ValueError:
                        rospy.logwarn("Invalid input. Please enter numeric values.")
                elif user_input == '2':
                    if self.goal_active:
                        self.stop_target()
                    else:
                        rospy.logwarn("No active goal to cancel.")
                elif user_input == '3':
                    rospy.loginfo("Exiting the goal manager.")
                    break
                else:
                    rospy.logwarn("Unrecognized command. Please try again.")

if __name__ == "__main__":
    try:
        node = GoalManager()
        node.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown safely.")

