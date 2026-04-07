#!/usr/bin/env python3
# This tells the system to run this file using Python 3

# rclpy = ROS 2 Python library (lets Python nodes talk to ROS)
import rclpy

# ActionClient = used to send requests (goals) to an Action Server
from rclpy.action import ActionClient

# Node = basic ROS unit (a running program in ROS)
from rclpy.node import Node

# Import our custom action definition (Goal, Feedback, Result)
from claw_driver_interfaces.action import DeploySensor


class DeploySensorClient(Node):
    """
    This class defines the ACTION CLIENT.

    Think of it like this:
    - This node asks: "Hey, deploy the sensor"
    - The server (claw_driver) does the work
    - The client listens for updates and final result
    """

    def __init__(self):
        # Initialize this node with name "deploy_sensor_client"
        super().__init__('deploy_sensor_client')

        # Create the Action Client
        self._action_client = ActionClient(
            self,                 # this node
            DeploySensor,         # action type (must match server)
            'deploy_sensor'       # name of action (must match server)
        )

    def send_goal(self, command='DEPLOY_SENSOR'):
        """
        This function sends a goal (request) to the action server.

        Example:
        command = "DEPLOY_SENSOR"
        """

        # Wait until the server is running
        self.get_logger().info('Waiting for deploy_sensor action server...')
        self._action_client.wait_for_server()

        # Create the goal message (this matches the .action file)
        goal_msg = DeploySensor.Goal()
        goal_msg.command = command  # set the command

        # Log what we're sending
        self.get_logger().info(f'Sending goal: {command}')

        # Send the goal asynchronously (non-blocking)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # called whenever feedback is received
        )

        # When the server responds (accept/reject), call this function
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        This function runs after the server responds to the goal request.

        It checks:
        - Did the server accept the goal?
        """

        # Get the goal handle (represents the running goal)
        goal_handle = future.result()

        # If server rejected the goal
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            rclpy.shutdown()
            return

        # If accepted
        self.get_logger().info('Goal accepted')

        # Ask for the final result (this is also async)
        result_future = goal_handle.get_result_async()

        # When result is ready, call this function
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        This function is called whenever the server sends feedback.

        Feedback = progress updates during execution
        """

        # Extract feedback data
        feedback = feedback_msg.feedback

        # Print it
        self.get_logger().info(f'Feedback: {feedback.current_step}')

    def get_result_callback(self, future):
        """
        This function runs when the server finishes the task.

        It gets the final result (success/failure)
        """

        # Extract result from future
        result = future.result().result

        # Print final result
        self.get_logger().info(
            f'Result: success={result.success}, message="{result.message}"'
        )

        # Shut down node after completion
        rclpy.shutdown()


def main(args=None):
    """
    Entry point of the program.
    """

    # Initialize ROS
    rclpy.init(args=args)

    # Create the client node
    node = DeploySensorClient()

    # Send the goal (start the action)
    node.send_goal()

    # Keep the node alive to receive feedback + result
    rclpy.spin(node)


# Run main() if this file is executed directly
if __name__ == '__main__':
    main()