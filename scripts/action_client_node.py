#!/usr/bin/env python3
"""Action Client Node for Robot Navigation

Provides interface for sending navigation goals to action server and publishes:
- Robot state (position/velocity)
- Last target coordinates

Action Clients:
    /reaching_goal (assignment_2_2024/PlanningAction): Navigation action server

Publishers:
    /robot_state (assignment_2_2024/RobotState): Current robot state
    /last_target (geometry_msgs/Point): Last sent target

Subscribers:
    /odom (nav_msgs/Odometry): For robot position/velocity data
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import RobotState
from geometry_msgs.msg import Point

class ActionClientNode:
    """Handles goal management and robot state monitoring
    
    Attributes:
        client (SimpleActionClient): Action client instance
        state_pub (Publisher): Robot state publisher
        last_target_pub (Publisher): Last target publisher
        robot_state (RobotState): Current state container
    """

    def __init__(self):
        """Initializes node, action client, publishers and subscribers"""
        rospy.init_node('action_client_node')
        
        # Action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        rospy.loginfo("Action server connected!")

        # Publisher for robot state
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        self.robot_state = RobotState()

        # Publisher for last target
        self.last_target_pub = rospy.Publisher('/last_target', Point, queue_size=10)

        # Subscriber for odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """Processes odometry data and publishes robot state
        
        Args:
            msg (Odometry): Incoming odometry message
        """
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        self.robot_state.vel_x = msg.twist.twist.linear.x
        self.robot_state.vel_z = msg.twist.twist.angular.z
        self.state_pub.publish(self.robot_state)

    def send_goal(self, x, y):
        """Sends new navigation goal to action server
        
        Args:
            x (float): Target x coordinate
            y (float): Target y coordinate
        """
        target_point = Point(x=x, y=y, z=0.0)
        self.last_target_pub.publish(target_point)

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)
        rospy.loginfo(f"Goal sent: ({x}, {y})")

    def feedback_callback(self, feedback):
        """Handles action feedback messages
        
        Args:
            feedback (PlanningFeedback): Feedback from action server
        """
        rospy.loginfo(f"Feedback received: {feedback}")

    def cancel_goal(self):
        """Cancels current navigation goal"""
        self.client.cancel_goal()
        rospy.loginfo("Goal canceled")

    def run(self):
        """Main node loop for user interaction"""
        rospy.loginfo("Action Client Node running...")
        while not rospy.is_shutdown():
            cmd = input("Enter 's' to send a goal, 'c' to cancel, or 'q' to quit: ")
            if cmd == 's':
                x = float(input("Enter target x: "))
                y = float(input("Enter target y: "))
                self.send_goal(x, y)
            elif cmd == 'c':
                self.cancel_goal()
            elif cmd == 'q':
                break

if __name__ == "__main__":
    node = ActionClientNode()
    node.run()
