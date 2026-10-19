ROS 2 Robot Control Telemetry — Integration Notes

1. Overview

This project simulates a robot using ROS 2. It supports velocity commands, robot telemetry, a DriveDistance action, and a ResetRobot service.

2. Node Graph

Record the actual nodes and interfaces discovered at runtime.

Robot simulator node: Maintains the robot's simulated position, orientation, velocity, and operating state.

Velocity subscriber: Receives geometry_msgs/msg/Twist commands on the configured velocity topic.

Timer callback: Updates the simulated position, publishes odometry, and updates action feedback when a goal is active.

DriveDistance action server: Validates goals, tracks progress, handles cancellation, and reports the result.

ResetRobot service server: Resets the robot to its initial pose when the reset is allowed.

3. Bug and Fixes

Bug 1

Symptom: robot state is in "MOVING" state eventhough the active_goal is inactive or does not exist.
Fix: added the active_goal check logic and moved the robot_state logic into update_and_send_pose method.

Bug 2

Symptom: robot_simulator_node node gets stopped when the action goal is completed.
Fix: added active_goal pointer reset logic.

Bug 3

Symptom: irregular response when action server recieves negative value or less than current position value.
Fix: added the condition checks to handle the edge cases.