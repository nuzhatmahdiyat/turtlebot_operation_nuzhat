# Turtlebot Autonomous Navigation & Override Controller

This project provides a ROS 2 autonomous obstacle-avoidance solution for the TurtleBot3, featuring a built-in manual override service.

## Overview

The system uses a LiDAR-based state machine (`forward` → `turn` → `reverse`) to navigate environments autonomously. It also exposes a ROS 2 service, `/set_direction`, allowing operators to manually take control of the robot's movement.

---

## 1. Step-by-Step Setup Instructions

### Installation

1. Create your workspace directory and move into it:

   ```bash
   mkdir -p ~/workspaces/turtlebot_operation_nuzhat/src
   cd ~/workspaces/turtlebot_operation_nuzhat
   git init
   ```

2. Create the interfaces package and controller package in the `src` directory as defined in the project structure.

### Building the Project

After creating your packages, build them and source the environment to make them discoverable by ROS 2:

```bash
cd ~/workspaces/turtlebot_operation_nuzhat
colcon build --packages-select obstacle_direction_interfaces
source install/setup.bash
colcon build --packages-select obstacle_direction_controller
source install/setup.bash
```

---

## 2. Detailed Command Explanation

This project requires specific ROS 2 commands to ensure the build system correctly links your custom services to your controller logic:

- `mkdir -p ~/workspaces/turtlebot_operation_nuzhat/src`: ROS 2 requires a specific workspace structure; the `src` directory is the standard location for all custom packages.
- `ros2 pkg create`: Initializes the packages; we use `--build-type ament_cmake` for interfaces to handle service generation, and `--build-type ament_python` for the controller to manage the Python node logic.
- `colcon build --packages-select [package_name]`: Compiles specific packages, generates the Python byte-code, and creates the C++ headers needed for service definitions.
- `source install/setup.bash`: Adds your package's directories to your system's `PATH`, allowing ROS 2 to locate your custom nodes and services.
- `ros2 run`: Uses the entry point defined in `setup.py` to execute your script as a managed ROS 2 process.
- `ros2 service call`: Acts as a client to send a structured request to your service, allowing for remote testing of your override logic.

---

## 3. How to Operate

### Running the Autonomous Node

Open a terminal and launch the autopilot:

```bash
source install/setup.bash
ros2 run obstacle_direction_controller direction_autopilot
```

### Manual Override

In a second terminal, send a command to override autonomous behavior. Replace `{direction}` with `forward`, `reverse`, `left`, or `right`:

```bash
source install/setup.bash
ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'left'}"
```

To return the robot to autonomous mode, call the service with `auto`:

```bash
ros2 service call /set_direction obstacle_direction_interfaces/srv/SetDirection "{direction: 'auto'}"
```

---

## 4. Expected Output

- **Terminal Logs:** Monitor the primary terminal for the robot's current state (e.g., `ACTION: FORWARD`) and real-time LiDAR distance readings (`F`, `L`, `R` values).
- **Service Response:** After a successful service call, the terminal will output:
  ```
  success=True, message='Direction override to [direction] accepted.'
  ```

---

## 5. Demo
[![Watch the Project Demo](https://img.youtube.com/vi/3aPIJC1wJWM/0.jpg)](https://www.youtube.com/watch?v=3aPIJC1wJWM)
