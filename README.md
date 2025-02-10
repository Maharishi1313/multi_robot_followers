# Multi-Robot Followers

This repository contains the ROS2 workspace for the simulation of multiple follower robots that follow a single leader bot.

## Dependencies
- **Nav2 Stack**
- **SLAM Toolbox**
- **TurtleBot3**

### Setting Up a New Workspace
To create a new ROS2 workspace and clone this repository, follow these steps:
```bash
# Create a new ROS2 workspace
mkdir -p ~/multi_robot_ws/src
cd ~/multi_robot_ws

# Clone this repository into the src folder
git clone https://github.com/Maharishi1313/multi_robot_followers.git src/multi_robot_followers

# Source the ROS2 setup script
source /opt/ros/humble/setup.bash  # Change 'humble' to your ROS2 version

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Set up the Gazebo model path
export GAZEBO_MODEL_PATH=$(pwd)/src/turtlebot3_multi_robot/models
```

## Overview
This workspace includes a custom ROS2 package named `leader_follower`, which contains the leader-follower control node. This control system is built upon the [multi-robot simulation repository](https://github.com/arshadlab/turtlebot3_multi_robot).

By default, it spawns four TurtleBot3 robots in Gazebo, all integrated with the Nav2 stack and organized using namespaces. This approach simplifies the navigation process.

### Leader-Follower Control Logic
The `lf_control` node subscribes to the `{namespace}/amcl_pose` topics of the leader and follower robots. It calculates the distance between them and commands the follower robot to:
- Move to the leader's position if the distance is above the threshold.
- Stop moving when within the threshold distance by setting its target position to its current location.

These commands are executed using the `navigate_to_pose` action feature from the Nav2 stack.

## What does the launch file do
The launch file `lf_final.launch.py` performs the following steps:
1. Launches the multi-robot simulation from the referenced repository.
2. Creates three instances of `lf_control` for follower robots, assigning them namespaces, effectively creating one leader and three followers.

Now, when the leader moves (either by setting a Nav2 goal or using `teleop_twist_keyboard`), the follower robots navigate using the Nav2 stack’s path planning and obstacle avoidance functionalities.

## Launching simulation
```
cd ~/multi_robot_ws
ros2 launch turtlebot3_multi_robot lf_final.launch.py
```
Open another terminal and run teleop_twist_keyboard node with remapped cmd_vel topic to move the leader bot
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -remap cmd_vel:=/tb1/cmd_vel
```
Now, move the leader bot to see the other bots following it.

## Possible Errors & Solutions
### 1. Error: Publishing `amcl_pose`
**Issue:** The `lf_control` node was not able to receive the initial pose of the bots (leader and follower). It only started working after the robots moved.

**Solution:** The subscribers were not receiving past messages. This was fixed by setting the QoS durability policy to **Transient Local**, ensuring compliance with the AMCL pose topic publisher.

### 2. Missing Map for Simulation
**Solution:** Used the ROS2 port of the `explore_lite` node (frontier search-based exploration) to autonomously generate the world map, necessary for Nav2 to function properly.

### 3. Goal Cancellation Issue
**Issue:** Initially, the approach was to cancel the follower's navigation goal when within the threshold distance of the leader, but it was difficult to implement effectively.

**Solution:** Instead of canceling the goal, the logic was changed to command the follower to move to its own current position when within the threshold distance. This approach works well in most cases but occasionally leads to collisions.

## Future Improvements
- Implement a **collision avoidance layer** using `costmap_filters`.
- Tune **DWA Planner** parameters to minimize rare collisions.

## References

- [Mobile-Swarm-Navigation](https://github.com/Loki-Silvres/Mobile-Swarm-Navigation): My previous work on Swarm Navigation.
- [multi-robot simulation repository](https://github.com/arshadlab/turtlebot3_multi_robot)
- [Explore package](https://github.com/robo-friends/m-explore-ros2)

