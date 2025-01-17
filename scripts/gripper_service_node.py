#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robotiq_gripper_ros2.srv import GripperCommand
from std_srvs.srv import Trigger
from robotiq_gripper_ros2 import robotiq_gripper


class RobotiqGripperServiceNode(Node):

    """
    A ROS 2 service node that connects to a Robotiq gripper but does NOT
    automatically activate it at startup. Instead, activation can be
    triggered via a separate service call.
    """

    def __init__(self):
        super().__init__('robotiq_gripper_service_node')

        # Declare parameters for IP and port
        self.declare_parameter('gripper_ip', '192.168.1.102')
        self.declare_parameter('gripper_port', 63352)

        ip = self.get_parameter('gripper_ip').value
        port = self.get_parameter('gripper_port').value

        # Create the RobotiqGripper instance
        self.get_logger().info("Creating RobotiqGripper instance...")
        self._gripper = robotiq_gripper.RobotiqGripper()

        # Connect to the gripper, but DO NOT ACTIVATE here
        self.get_logger().info(
            f"Connecting to RobotiqGripper at {ip}:{port}...")
        self._gripper.connect(ip, port)

        # Create the services:
        # 1) Service to activate the gripper
        self._activate_srv = self.create_service(
            Trigger,
            'activate_gripper',
            self.activate_gripper_callback
        )

        # 2) Service to command the gripper (open/close)
        self._command_srv = self.create_service(
            GripperCommand,
            'robotiq_gripper_command',
            self.gripper_command_callback
        )

        self.get_logger().info("RobotiqGripperServiceNode is ready.")
        self._activated = False  # Track activation state

    def activate_gripper_callback(self, request, response):
        """
        Callback for the 'activate_gripper' service.
        Tries to activate the gripper and sets response.success accordingly.
        """
        try:
            self.get_logger().info("Activating the gripper...")
            self._gripper.activate()
            self._activated = True
            response.success = True
            response.message = "Gripper activated successfully."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to activate gripper: {e}"
            self.get_logger().error(response.message)
        return response

    def gripper_command_callback(self, request, response):
        """
        Callback for the 'robotiq_gripper_command' service.
        Moves the gripper to the requested position, speed, and force.
        """
        # If you want to ensure the gripper is activated before commanding, do:
        if not self._activated:
            self.get_logger().warn("Gripper is not activated yet. Call 'activate_gripper' first.")
            response.success = False
            response.final_position = -1
            return response

        try:
            self.get_logger().info(
                f"Received gripper command: pos={request.position}, speed={
                    request.speed}, force={request.force}"
            )

            # Move the gripper
            self._gripper.move_and_wait_for_pos(
                request.position,
                request.speed,
                request.force
            )

            # Fill in the response
            response.final_position = self._gripper.get_current_position()
            response.success = True

            self.get_logger().info(
                f"Gripper move success. Final position: {
                    response.final_position}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to move the gripper: {e}")
            response.success = False
            response.final_position = -1

        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
