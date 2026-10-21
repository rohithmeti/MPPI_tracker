# Run Commands

Tested on Ubuntu 22.04 with ROS 2 Humble.

## 1. Put the package in a workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/rohithmeti/MPPI_tracker.git
```

## 2. Install all dependencies (Nav2 MPPI, TurtleBot3 Gazebo, ...)

```bash
cd ~/ros2_ws
sudo rosdep init                   # only if rosdep was never initialised
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 3. Build and run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch nav sim_bringup.py
```

Gazebo, the robot, MPPI, the path smoother, the bridge and RViz all start, and the robot drives the route.
The first Gazebo start can take a minute while it loads models.

## Stop

Press `Ctrl+C` in the launch terminal, or from another terminal:

```bash
pkill -INT -f "ros2 launch nav sim_bringup"
```
