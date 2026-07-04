# Project Kratos
## Autonomous Subsystem

## Project Overview
This repository is the ROS2 workspace contians 
    **Turtle_controller**
    **coordinate follower**
    **Mission control GUI** 
packages respectively.

## Package 1: turtle_controller

### Overview
The `turtle_controller` is the first package of this workspace. It publishes velocity commands to guide the robot in a continuous circular trajectory. This serves as the initial validation step to ensure the simulator's kinematic model and the ROS 2 transport layer are communicating correctly.

### How It Works
    Creates velocity commands repeatedly.
    Sends a forward speed and a turning speed.
    Publishes these commands to the /cmd_vel topic.
    The simulator receives these commands and moves the robot in a circular path.
### How to Run This Package
    cd ~/induction_ws
    source install/setup.bash

    ros2 run turtle_controller draw_circle


## Package 2: coordinate_follower

### Overview
The `coordinate_follower` package implements closed-loop, autonomous waypoint navigation. Serving as an on-board mission client for the Project Kratos rover, this package reads an arbitrary sequence of space-separated Cartesian coordinates from an external file and dispatches them sequentially to the Nav2 navigation stack via an asynchronous action client interface.

### How It Works
    Reads coordinates from a text file.
    Sends each coordinate to the Nav2 navigation system.
    Displays the remaining distance while the robot is moving.
    After reaching one waypoint, it automatically moves to the next.
    If a waypoint cannot be reached, it skips it and continues with the remaining points.

### Example of waypoints.txt
    2.0 1.5
    0.5 3.0
    -1.0 2.0

### How to Run This Package
To execute the autonomous coordinate follower alongside the simulation stack:

1. **Verify your `waypoints.txt` file is formatted cleanly:**
   ```text
   2.0 1.5
   0.5 3.0
   -1.0 2.0

2. **Launch the Node with the parameter argument:**
    cd ~/induction_ws
    source install/setup.bash
    ros2 run coordinate_follower mission_client --ros-args -p waypoint_file:=waypoints.txt

## Package 3: mission_control_gui

### Overview
The `mission_control_gui` package serves as the final integration layer for Project Kratos. It replaces the terminal-based execution of the waypoint client with a high-fidelity operator dashboard built using **PyQt5**. This package embeds a ROS 2 node directly into the graphical interface window, combining local coordinate validation, sequential Nav2 action routing, and a real-time telemetry array into a single ground control asset.

### Features
    Enter coordinates through the GUI.
    Checks whether the entered coordinates are valid.
    Sends waypoints to the robot one by one.
    Displays the remaining distance while the robot is moving.
    Shows the robot's:
    Current position
    Speed
    Heading (direction)

### How It Works
    Uses PyQt5 to create the interface.
    Uses a timer so that both the GUI and ROS 2 can run together without freezing.
    Validates user input before sending coordinates.
    Sends each waypoint to Nav2.
    Waits until the robot reaches the current waypoint before sending the next one.

### How to Run This Package

1. **Start Nav2 simulation**
    cd ~/genesis_sim
    source ~/induction_wsinstall/setup.bash
    ros2 launch launch_nav2.py

2. **Start Turtlebot simulation**
    cd ~/genesis_sim
    python3 turtlebot_sim.py

3. **Start the GUI**
    cd ~/induction_ws
    source install/setup.bash
    ros2 run mission_control_gui main