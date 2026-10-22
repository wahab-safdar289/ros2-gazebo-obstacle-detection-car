# ROS 2 Obstacle-Avoiding Car (Gazebo)

A 4-wheel skid-steer car simulated in Gazebo that drives itself around a map full of
obstacles using a lidar. It also carries a camera, which publishes images but isn't
used for detection yet. Built with ROS 2 Lyrical, Gazebo Jetty and `ros2_control`.

> **Status:** work in progress. The simulation, sensors, controllers and the
> control nodes start up correctly.

## How it works

```
Gazebo lidar ──► /scan ──┬─► obstacle_avoider ──► /cmd_vel_avoid ──┐
Gazebo camera ─► /camera/image_raw                                  ▼
                         └─────────────────────► safety_gate ──► /diff_drive_controller/cmd_vel
                                                                    │
                                          diff_drive_controller (ros2_control + gz_ros2_control)
                                                                    │
                                                          4 wheel joints in Gazebo
```

- **camera_joint** is 1.5 cm and **lidar_joint** is 0.5 cm ahead of `base_link`.
- **obstacle_avoider** reads `/scan` and runs a small state machine
  (cruise, turn, reverse, halt) that steers away from obstacles.
- **safety_gate** sits between the avoider and the wheels. It reads `/scan` on its own,
  limits speed and acceleration, blocks forward motion near obstacles, and stops the car
  if the lidar or the commands go stale.
- The launch file starts everything step by step and stops if any step fails.

## Packages

| Package | Purpose |
|---|---|
| `car_description` | URDF/xacro model, `ros2_control` and controller config, Gazebo-ROS bridge config, RViz layout |
| `car_gazebo` | Obstacle world (`obstacle_world.sdf`) and the main `bringup.launch.py` |
| `car_control` | `obstacle_avoider` and `safety_gate` nodes, their parameters, unit tests |

## Requirements

- ROS 2 Lyrical with Gazebo Jetty (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`)
- `ros2_controllers` (`diff_drive_controller`, `joint_state_broadcaster`), `xacro`, `rviz2`

## Build

```bash
mkdir -p ~/car_ws/src && cd ~/car_ws/src
git clone https://github.com/<your-username>/<repo-name>.git
cd ~/car_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch car_gazebo bringup.launch.py
```

| Argument | Default | Meaning |
|---|---|---|
| `start_control` | `true` | `false` starts the simulation without the autonomy nodes |
| `rviz` | `true` | start RViz |
| `x`, `y`, `z`, `yaw` | `0 0 0.05 0` | spawn pose |
| `gz_flags` | `-r -v 2` | flags for `gz sim` (for example `-r -s -v 2` for no GUI) |
| `world` | bundled world | path to another SDF world |

Drive by hand (with `start_control:=false`):

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}}}"
```

## Tuning

All avoidance and safety numbers are in `car_control/config/car_control_params.yaml`.
Wheel and speed limits are in `car_description/config/controllers.yaml`.

## Tests

```bash
python3 -m pytest car_control/test/test_obstacle_logic.py -v
```

## Known issues

- Gazebo can be very slow to start when it renders with the CPU (for example under WSL).
  The controller spawners wait up to 120 s for this reason.
- Gazebo may not exit cleanly after a crash. Run `scripts/kill_sim.sh` before relaunching.

## License

Apache-2.0
