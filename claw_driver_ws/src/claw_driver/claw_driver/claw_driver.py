import time

# rclpy = ROS 2 Python client library (lets Python talk to ROS)
import rclpy

# These are specific tools for creating an "Action Server"
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Node = basic building block of ROS (like a program that runs in ROS)
from rclpy.node import Node

# This is our custom action definition (Goal, Feedback, Result)
from claw_driver_interfaces.action import DeploySensor


class ClawDriverActionServer(Node):
    """
    This class defines the ACTION SERVER.
    - Another node (client) sends a request: "deploy the sensor"
    - This server receives it, performs the task, sends updates, and returns a result
    """

    def __init__(self):
        # Initialize this node with the name "claw_driver"
        super().__init__('claw_driver')

        # Create the Action Server
        self._action_server = ActionServer(
            self,                   # This node
            DeploySensor,           # The action type (Goal/Result/Feedback structure)
            'deploy_sensor',        # The name of the action topic
            execute_callback=self.execute_callback,   # What to do when a goal runs
            goal_callback=self.goal_callback,         # What to do when a goal is received
            cancel_callback=self.cancel_callback      # What to do if client cancels
        )

        self.get_logger().info('claw_driver action server started')

    def goal_callback(self, goal_request):
        """
        Called when a client sends a goal.

        Example:
        mission.py sends: "DEPLOY_SENSOR"
        """

        self.get_logger().info(
            f'Received goal request: command="{goal_request.command}"'
        )

        # Only accept valid commands
        if goal_request.command != 'DEPLOY_SENSOR':
            self.get_logger().warn('Rejecting unknown claw command')
            return GoalResponse.REJECT  # reject the goal

        return GoalResponse.ACCEPT  # accept the goal

    def cancel_callback(self, goal_handle):
        """
        Called if the client wants to cancel the action mid-way.
        """

        self.get_logger().info('Received cancel request')

        # Allow cancellation
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Once a goal is accepted, this function:
        - performs the task
        - sends feedback updates
        - returns a final result
        """

        self.get_logger().info('Executing claw deployment action')

        # This will hold the final result we send back
        result = DeploySensor.Result()

        try:
            # STEP 1: Send command to Pico
            self.publish_feedback(goal_handle, 'Sending serial command to Pico')
            self.get_logger().info('TODO: send DEPLOY_SENSOR over UART')
            time.sleep(1)  # simulate delay

            # Check if user canceled during execution
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'CANCELED'
                return result

            # STEP 2: Wait for Pico to respond
            self.publish_feedback(goal_handle, 'Waiting for Pico acknowledgment')
            self.get_logger().info('TODO: wait for Pico ACK')
            time.sleep(1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'CANCELED'
                return result

            # STEP 3: Pico runs claw movement
            self.publish_feedback(goal_handle, 'Pico running claw sequence')
            self.get_logger().info('TODO: monitor serial status from Pico')
            time.sleep(2)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'CANCELED'
                return result

            # STEP 4: Done
            self.publish_feedback(goal_handle, 'Deployment complete')

            # Mark goal as successful
            goal_handle.succeed()

            result.success = True
            result.message = 'COMPLETE'

            self.get_logger().info('Claw deployment completed successfully')
            return result

        except Exception as e:
            # If something crashes, mark action as failed
            self.get_logger().error(f'Claw deployment failed: {e}')
            goal_handle.abort()

            result.success = False
            result.message = f'FAILED: {str(e)}'
            return result

    def publish_feedback(self, goal_handle, step_text):
        """
        Helper function to send feedback to the client.

        Feedback = updates while the task is running
        """

        feedback_msg = DeploySensor.Feedback()
        feedback_msg.current_step = step_text

        # Send feedback back to the client
        goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    """
    This starts the ROS node.
    """

    # Initialize ROS
    rclpy.init(args=args)

    # Create the node (start the action server)
    node = ClawDriverActionServer()

    # Keep the node running (listening for goals)
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()