# Ignition Gazebo setup

## Requirements

- Ubuntu 22.04 or higher
- ROS2 Humble

This setup tutorial assumes that ROS2 is already installed and is setup to run nodes.

## Install useful tools

- `sudo apt update && sudo apt upgrade`
```bash
sudo apt install ros-humble-turtlebot4-description ros-humble-turtlebot4-msgs ros-humble-turtlebot4-navigation ros-humble-turtlebot4-node ros-dev-tools
```

## Install Ignition Gazebo fortress

```bash
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

## Install the turtlebot 4 simulator

`sudo apt install ros-humble-turtlebot4-simulator`

## Ignition bringup

`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py`

### If the model doesn't load into gazebo but everything else loads

- Don't forget to source your bashrc:

`source ~/.bashrc`

- Try first running the simulation with the lite model:

`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite`

Then try the original ignition again

If you are encountering the message being printed every ~1 second:

```
Waiting messages on topic [robot_description].
Waiting messages on topic [standard_dock_description].
```

- Try running the ignition script without the turtlebot:

`ros2 launch turtlebot4_ignition_bringup ignition.launch.py`

Then try the original ignition again

- Try specifying the coordinates to (0, 0, 0):

`ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py x:=0 y:=0 z:=0`

- Check if the topics are being created using `ros2 topic list`

If issue persist, go see https://github.com/turtlebot/turtlebot4/issues/351

## Visualisation with Rviz

- `sudo apt install ros-humble-turtlebot4-desktop`


## Launch Rviz 

If ignition is currently running (don't forget to turn on the time (button in the botton left corner)), run:

- `ros2 launch turtlebot4_viz view_robot.launch.py`

The "map" frame in ROS typically refers to the global reference frame in the environment. This frame is often used for localization and navigation purposes, as it provides a stable reference point for robots to navigate within a known environment.

The base_link frame represents the reference frame attached to the robot's base or chassis. Its used for navigation algorithms, such as path planning and obstacle avoidance, typically plan trajectories and control robot motion relative to the base_link frame.

