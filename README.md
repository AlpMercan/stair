# Turtlebot3 Stair Detection Layer

A custom costmap layer for ROS that simulates stair detection for Turtlebot3. This package adds a costmap layer that creates obstacles in front of the robot based on sensor input.

## Features

- Custom costmap layer that integrates with Turtlebot3's navigation stack
- Creates obstacle markings in front of the robot when stairs are detected
- Maintains obstacle positions until navigation goal is completed or aborted
- Includes a mock sensor node for testing purposes

## Prerequisites

- ROS Noetic
- Turtlebot3 packages
- Ubuntu 20.04

```bash
# Install Turtlebot3 packages
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-turtlebot3-simulations
```

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/AlpMercan/stair_costmap.git
git clone https://github.com/ros-planning/navigation.git
add the navigations position of ros planing to your stari_costmaps cmakelist
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:
```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

1. Launch Turtlebot3 simulation:
```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

2. Launch navigation stack:
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

3. Run the mock sensor node for testing:
```bash
rosrun stair mock_sensor.py
```

## Testing

The mock sensor node publishes a sequence of values [10, 10, 10, 60, 10, 10, 10] to simulate stair detection:
- Values below threshold (50.0): Safe to navigate
- Values above threshold (50.0): Stairs detected

## Parameters

The costmap layer can be configured with the following parameters:
- `threshold` (default: 50.0): Detection threshold
- `costmap_value` (default: 254): Cost value for detected obstacles
- `region_size` (default: 0.3m): Size of the marked region
- `forward_distance` (default: 0.5m): Distance in front of robot to mark obstacles

## Node Information

### Published Topics
- None directly (modifies costmap)

### Subscribed Topics
- `/stair_detection` (std_msgs/Float32): Sensor input Its mocked for this case
- `/move_base/status` (actionlib_msgs/GoalStatusArray): Navigation status

## Architecture

The package consists of two main components:
1. StairLayer: Custom costmap layer that integrates with move_base
2. Mock Sensor: Python node that simulates sensor data for testing

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

ALP MERCAN

