# toio_gazebo

## Introduction

This is a ROS 2 Package to develop package of [toio](https://toio.io/) using Gazebo.



https://github.com/user-attachments/assets/63909751-a5ec-49ca-8c24-c2067247fe8e



## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Build

```bash
sudo apt update
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/atinfinity/toio_description.git
git clone https://github.com/atinfinity/toio_gazebo.git
cd ..
rosdep install -y -i --from-paths src
colcon build --symlink-install
source ~/dev_ws/install/setup.bash
```

## Launch Gazebo

```bash
ros2 launch toio_gazebo simulation.launch.py
```
