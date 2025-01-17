#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from robotiq_gripper_ros2.srv import GripperCommand
from robotiq_gripper_ros2 import robotiq_gripper


class RobotiqGripperServiceNode(Node):
    """
    A ROS 2 service node that interfaces with a Robotiq gripper.
    It exposes a service to move the gripper to the requested position, speed, and force.
    """

    def __init__(self):
        super().__init__('robotiq_gripper_service_node')

        # Declare parameters or hardcode them:
        # The IP and port of the gripper
        self.declare_parameter('gripper_ip', '192.168.1.102')
        self.declare_parameter('gripper_port', 63352)

        ip = self.get_parameter('gripper_ip').value
        port = self.get_parameter('gripper_port').value

        # Create the RobotiqGripper instance
        self.get_logger().info("Creating RobotiqGripper instance...")
        self._gripper = robotiq_gripper.RobotiqGripper()

        # Connect to the gripper
        self.get_logger().info(
            f"Connecting to RobotiqGripper at {ip}:{port}...")
        self._gripper.connect(ip, port)

        # Activate the gripper
        self.get_logger().info("Activating the gripper...")
        self._gripper.activate()

        # Create the service
        self._srv = self.create_service(
            GripperCommand,
            'robotiq_gripper_command',  # Service name
            self.gripper_command_callback
        )

        self.get_logger().info("RobotiqGripperServiceNode is ready.")

    def gripper_command_callback(self, request, response):
        """
        Callback that will be triggered when the service is called.
        - request.position
        - request.speed
        - request.force
        """
        try:
            self.get_logger().info(
                f"Received gripper command: pos={request.position}, "
                f"speed={request.speed}, force={request.force}"
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
            response.final_position = -1  # Indicate an error or invalid position

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

