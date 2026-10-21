# MPPI Waypoint Tracker with Obstacle Avoidance

ROS 2 Humble package for a TurtleBot3 (Gazebo). It takes waypoints from a CSV file, turns them into a
smooth path, and tracks the path with the Nav2 **MPPI** controller. When an obstacle is on the path,
the robot drives around it using the 2D lidar and comes back to the path.

Based on [manojkarnekar/smoothing-rpp](https://github.com/manojkarnekar/smoothing-rpp), where a
Pure Pursuit tracker follows the path and stops when an obstacle is in front.

## Tasks

| # | Task | Done by |
|---|---|---|
| 1 | Use an MPPI tracker instead of Pure Pursuit | Nav2 `controller_server` with the MPPI plugin (`config/mppi.yaml`) |
| 2 | Generate a smooth path from the waypoints and give it to MPPI | `nodes/path_smoother.py` (cubic spline) and `nodes/path_to_mppi.py` |
| 3 | Go around obstacles using the 2D lidar and track the original path again | MPPI + local costmap built from `/scan` |
| 4 | Handle static and dynamic obstacles | the costmap is updated from every scan; obstacles added or removed while driving are handled |

## How it works

```
waypoint.csv ──► path_smoother.py ──► /path ──► path_to_mppi.py ──► controller_server ──► /cmd_vel ──► TurtleBot3
 (4 points)      cubic spline        /waypoints   one piece per       Rotation Shim + MPPI
                                                  waypoint            local costmap ◄── /scan (lidar)
```

1. **CSV → smoother** (`nodes/path_smoother.py`)
   Reads the waypoints from `waypoints/waypoint.csv` and fits a **cubic spline** through them, so the
   path passes exactly through every waypoint with no sharp corners. It keeps a point every 10 cm, gives
   each point a heading, and publishes the path on `/path` and the raw waypoints on `/waypoints`.
   The spline code is `spline/cubic_spline_planner.py` from
   [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (MIT), unchanged.

2. **Smoother → MPPI** (`nodes/path_to_mppi.py`)
   The Nav2 controller does not read a path topic; it takes a path through the `FollowPath` action.
   This node is the bridge. It cuts `/path` into one piece per waypoint and sends the pieces one after
   another, so every waypoint is really reached (within 5 cm) before the robot heads to the next one.
   If MPPI gives up (for example when blocked), it re-sends the rest of the piece. If a waypoint stays
   unreachable for 60 s (for example an obstacle sitting on it), it reports `MISSION FAILED` and the
   robot stays stopped.

3. **MPPI drives** (`config/mppi.yaml`)
   20 times a second MPPI imagines 2000 short future motions (4 s each), scores them (stay near the
   path, reach the waypoint, keep away from obstacles) and sends the best motion to `/cmd_vel`.
   Obstacles come from a 4 m × 4 m costmap around the robot, filled from the lidar. The Rotation Shim
   in front of MPPI makes the robot turn on the spot when the next path piece points the other way
   (the sharp turns at the waypoints).

4. **Launch** (`launch/sim_bringup.py`)
   One command starts Gazebo with the TurtleBot3, the controller, the smoother, the bridge and RViz.

## Old vs new

| | Original (Pure Pursuit) | This package (MPPI) |
|---|---|---|
| Path | corners cut by 0.76 m, kinks, no headings | smooth spline through every waypoint, with headings |
| Obstacle on the path | stops and waits until it is removed | drives around it and returns to the path |
| Sudden obstacle while driving | stops | drives around it |
| Waypoints | not checked | each one reached within ~5 cm |
| Obstacle on the final goal | waits forever | stops without touching it, reports `MISSION FAILED` after 60 s |

## MPPI settings that matter

All other values are standard Nav2 values. Each change below was tested in simulation.

| Setting | Value | Why |
|---|---|---|
| `PathAlignCritic` / `PathFollowCritic` weight | 2 / 2 (Nav2 example: 14 / 5) | with a strong pull to the path the robot stopped in front of a 1 m obstacle; a weaker pull lets it leave the path to go around |
| `PathAngleCritic` | off | turning towards the path is done by the Rotation Shim |
| `xy_goal_tolerance`, `GoalCritic` weight | 0.05 m, 10 | each waypoint reached within ~5 cm |
| Rotation Shim | on | turns on the spot at sharp waypoints instead of driving a wide loop |
| `time_steps` | 80 (4 s) | shorter look-ahead gave wider, slower detours |
| `inflation_radius`, `cost_scaling_factor` | 0.35, 3.0 | keeps ~0.25 m from obstacles; a smaller inflation made the robot hit a box |
| `inf_is_valid` (costmap) | true | lidar beams that hit nothing clear old obstacles, so removed obstacles disappear |
| path point spacing | 0.1 m | MPPI counts its look-ahead in path points; this spacing matches the settings |

## Results (Gazebo)

Full route with a 1 m block and a 30 cm box on the path, and a 40 cm box dropped in front of the
moving robot:

| | Result |
|---|---|
| Obstacles passed | all three, no stops or aborts |
| Waypoint error | 0.02 – 0.05 m at (2, 6), (4, −6) and (5, 4) |
| Closest distance to obstacles | ~0.25 m |
| Time for the whole route | ~150 – 190 s |

## How to run

Full steps: [RUN_COMMANDS.md](RUN_COMMANDS.md). In short (Ubuntu 22.04, ROS 2 Humble):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/rohithmeti/MPPI_tracker.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch nav sim_bringup.py
```

In RViz: red = CSV waypoints, green = smooth path, coloured lines = MPPI's imagined motions.

### Adding obstacles

In Gazebo use the box / cylinder buttons in the top toolbar and drop them on the path, before or
while the robot drives. Or from a terminal, for example a 50 cm box on the second leg of the path:

```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'box1', xml: '<sdf version=\"1.6\"><model name=\"box1\"><static>true</static><link name=\"l\"><collision name=\"c\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: 3.3, y: -0.09, z: 0.25}}}"
```

Remove it with:

```bash
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box1'}"
```

## Files

```
config/mppi.yaml               controller_server, MPPI, Rotation Shim and local costmap settings
config/sim.rviz                RViz layout
launch/sim_bringup.py          starts everything (the old Pure Pursuit node is kept, commented out)
nodes/path_smoother.py         CSV -> cubic spline -> /path, /waypoints
nodes/path_to_mppi.py          /path -> FollowPath action, one piece per waypoint
spline/cubic_spline_planner.py cubic spline (PythonRobotics)
waypoints/waypoint.csv         the waypoints
src/smoothing.cpp              original smoother (not used, kept for comparison)
src/pure_pursuit.cpp           original Pure Pursuit tracker (not used, kept for comparison)
```

## Limitations

- Tested in simulation only. The robot position comes from wheel odometry, which drifts on a real
  robot; there is no map or localization.
- Obstacles up to about 1 m on the path were tested. Much larger blockages (walls, dead ends) need a
  global planner that re-plans the route; MPPI is a local controller.
- At very sharp waypoints the robot stops and turns on the spot before continuing.

## References

- [Nav2 MPPI controller](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- [Nav2 Rotation Shim controller](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/)
- [PythonRobotics cubic spline planner](https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/cubic_spline/cubic_spline.html)
- Ravankar et al., *Path Smoothing Techniques in Robot Navigation*, Sensors 2018
