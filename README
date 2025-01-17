# Robotiq Gripper ROS 2 Package

This package provides a **ROS 2** service node for controlling a Robotiq gripper over a socket connection.

> [!TIP]
> Works with Robotiq_grippers UR Cap.

> [!CAUTION]
> You might not be able to leverage existing robotiq drivers,depending on implementation.



## Contents

- `srv/GripperCommand.srv`: Custom service definition (position, speed, force -> success, final_position)
- `robotiq_service_node.py`: Main ROS 2 service node that controls the Robotiq gripper
- `robotiq_gripper.py`: Python library for sending and receiving socket commands to the gripper

## Requirements

- ROS 2 (Humble, Iron, or newer)
- Python 3
- Network connection to the Robotiq gripper or UR controller running the Robotiq URCap in socket mode

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <URL to your repository>
   ```

2. Install dependencies as needed (e.g., `rclpy`, `rosidl_default_generators`, etc.)

3. Build and source:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### 1. Run the Service Node

```bash
ros2 run <package_name> robotiq_service_node \
    --ros-args \
    -p gripper_ip:=192.168.1.102 \
    -p gripper_port:=63352
```

- `gripper_ip` is the IP address of the gripper or UR controller
- `gripper_port` is the TCP port (default `63352`, unless you have changed it)

### 2. Call the Service

Use the following service call to open or close the gripper:

```bash
ros2 service call /robotiq_gripper_command <package_name>/srv/GripperCommand \
    "{position: 255, speed: 150, force: 200}"
```

Parameters:
- `position`: Desired position (0 = fully open, 255 = fully closed)
- `speed`: Movement speed (0-255)
- `force`: Grip force (0-255)

### 3. Check Output

You should see:
```
success: true
final_position: 255
```

This indicates the gripper has moved successfully and is at position 255.

## Troubleshooting

### ValueError: invalid literal for int() with base 10: '?'

1. Confirm the IP and port are correct
2. Verify the gripper's firmware supports the `SET/GET` socket commands
3. Make sure the gripper is in socket-control mode
4. Try `nc 192.168.1.102 63352` and manually send `GET STA`. A valid response should look like `STA 3`

### Connection Errors

- Check network connectivity (e.g., `ping 192.168.1.102`)
- Ensure any firewall rules are disabled or allow traffic on the specified port


## Contributing

Contributions are welcome! Feel free to open issues or pull requests to share your improvements.