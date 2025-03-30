# IntegratedPipeline - SeDriCa-BatMobile

This directory contains the ROS2 packages for the target detection, tracking, and cannon control system of the BatMobile platform.

## `batmobile_integrated` Package

### Overview

The `batmobile_integrated` package implements a state machine architecture for autonomous target acquisition and cannon control. The system progresses through four main states:

1. **Scanning**: The robot moves in a pattern combining rotation and linear movement to efficiently search for targets
2. **Turning**: Once a target is found, the robot adjusts its orientation to face the target
3. **Moving**: The robot approaches the target to an optimal shooting distance
4. **Shooting**: The robot aims the cannon and fires when in position

### System Architecture

The package uses a single C++ executable (`coordinates_receiver`) that internally manages two ROS2 nodes:
- `coordinate_subscriber`: Processes incoming target coordinates
- `control_publisher`: Generates and publishes control commands

These nodes are managed by a ROS2 executor for efficient concurrent operation.

### Key Components

#### Custom Messages

The package defines a custom message type for robot control:

**`ControlInstructions.msg`**:
```
float32 linear_speed   # Linear speed for forward/backward movement
float32 angular_speed  # Angular speed for turning left/right
bool shoot_cannon      # Flag to trigger cannon firing
float32 cannon_angle   # Vertical angle for cannon aiming (in radians)
```

#### Turning Logic

The turning logic calculates the optimal direction and speed to face the target:
- Computes angle difference between current heading and target position
- Normalizes angle difference to [-π, π]
- Determines turn direction based on the sign of the angle difference
- Adjusts turn speed based on angle magnitude

#### Movement Control

The movement control logic:
- Calculates optimal stopping position at a fixed distance from the target
- Modulates speed based on distance (slows down as it approaches)
- Minimizes oscillation with threshold-based control

#### Cannon Control

The cannon control includes:
- Vertical angle calculation based on target height
- Settling time delay to ensure stable positioning before firing
- Shooting confirmation flag to trigger the firing mechanism

### ROS2 Topics

- **Subscribed Topics**:
  - `/get_point` (geometry_msgs/msg/Point): Target coordinates from perception system

- **Published Topics**:
  - `/Control_instruction` (batmobile_integrated/msg/ControlInstructions): Control commands for robot movement and cannon

### Using the Package

#### Building

```bash
# From the IntegratedPipeline directory
colcon build --packages-select batmobile_integrated
```

#### Running

```bash
# Source the workspace
source install/setup.bash

# Launch the system
ros2 launch batmobile_integrated batmobile.launch.py
```

#### Testing

You can test the system by publishing target coordinates:

```bash
# Publish a target point
ros2 topic pub /get_point geometry_msgs/msg/Point "{x: 1.0, y: 0.5, z: 2.0}"
```

You can monitor the control commands:

```bash
# Monitor control commands
ros2 topic echo /Control_instruction
```

### Parameters

The system uses the following parameters (currently hardcoded, future work to make them configurable):

- `threshold`: Distance threshold for approach (0.6 meters)
- `epsilonA`: Angular precision threshold (0.1 radians)
- `epsilonB`: Positional precision threshold (0.1 meters)
- `linear_speed`: Maximum linear speed (1.0 m/s)
- `angular_speed`: Maximum angular speed (0.5 rad/s)
- `CANNON_SETTLE_TIME_MS`: Cannon settling time (1000 ms)

## Future Work

- Implement obstacle avoidance during movement
- Add multiple target prioritization logic
- Create visualization tools for system state monitoring 