# Ignition Gazebo setup

## Requirements

- Ubuntu 22.04 or higher
- ROS2 Humble

This setup tutorial assumes that ROS2 is already installed and is setup to build and run nodes.

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

# On the real turtlebot 4

## Requirements

- Ubuntu 22.04 or higher
- ROS2 Humble
- Turtlebot 4 
- Rviz

## Install navigation

If you have setup the discovery correctly, either with simple discovegit@gitlab.com:kdg-ti/the-lab/teams-23-24/bytebot/navigation.gitry or with discovery server, you should be seeing all topics of the turtle bot when running `ros2 topic list`. If you don't have those, refer to https://turtlebot.github.io/turtlebot4-user-manual/setup/simple_discovery.html or https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html

if everything is setup correctly, run `sudo apt install ros-humble-turtlebot4-navigation`

## Generate your first map

- `ros2 launch turtlebot4_navigation slam.launch.py` to run the SLAM module
- in another terminal `ros2 launch turtlebot4_viz view_robot.launch.py` to see the generated map in rviz

- drive your turtlebot around so that the whole map is generated
- save the map using ```ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"``` change map_name to the name you want to give to your map, that map will be saved as one pgm file and one yaml file in your current directory.

## Navigate your turtlebot through the map

Now that the map is generated, you naturally want your turtlebot to navigate through the environment:

- `ros2 launch turtlebot4_navigation localization.launch.py map:=your_map.yaml` replace 'your_map' with the path of the map you created
- run nav2 in another terminal `ros2 launch turtlebot4_navigation nav2.launch.py`
- run rviz in a third terminal `ros2 launch turtlebot4_viz view_robot.launch.py`

## Run the navigation module

### Clone the Repository

- run `git clone git@gitlab.com:kdg-ti/the-lab/teams-23-24/bytebot/navigation.git`
This will have created the `navigation` directory, run `ls` to make sure it's there, then go in it using `cd navigation`
- run `colcon build`
If there are no issues when building, you are ready for the next step

### Run the nav module

When all three terminals from the "Navigate your turtlebot through the map" are running, you should be able to run the navigation module
- in a 4th terminal, run `ros2 launch nav_controller nav_points.launch.py`
This will run 4 nodes:

#### nav_points
- gives you the possibility to give it commands (points to go to) and the turtlebot will go to those points.
- First you need to undock it: `ros2 topic pub /pose_listener std_msgs/msg/String "data: 'undock'" -1`
- then you can use the same command to give it a point to go to: `ros2 topic pub /pose_listener std_msgs/msg/String "data: 'entrance'" -1`
- To dock it again, run: `ros2 topic pub /pose_listener std_msgs/msg/String "data: 'dock'" -1`

#### show_all_commands
- If you want to know all the commands you can use: `ros2 topic pub /commands std_msgs/msg/String "data: 'list'" -1`

#### clicked_point_listener
- If you want to add a new point, you first need to know the coordinates of that point. To do that, you can go on rviz, select "publish point" on the top menu. After that you can click anywhere on the map to select a point. that point will then be shown on the same terminal where u launched the nav_controller

#### add_waypoint
- to add a new waypoint, you need to run this command: `ros2 topic pub /add_waypoint nav_module_interfaces/msg/TargetCoordinates '{name: "my_new_waypoint", coordinates: {x: -7.703, y: -4.424, z: 0.0}}' -1` you can change the name of the waypoint and the coordinates. We recommend to have z always at 0.