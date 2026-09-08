# mecharm_sim

ROS 2 Humble + Gazebo Sim package for a mechArm 270 pick-and-place task.

The simulation starts a table scene, spawns a mechArm 270, moves through calibrated pick-and-place waypoints, closes/opens the gripper, and verifies that the ball is at the configured target position. MoveIt pose planning is available as an opt-in mode and its complete time-parameterized trajectory is tracked through `ros2_control`.

## Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-control-msgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher
```

## Build

```bash
cd /mnt/textop_big/mecharm
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mycobot_description mecharm_moveit_config mecharm_sim
source install/setup.bash
```

## Run

```bash
ros2 launch mecharm_sim sim.launch.py
```

To plan the end-effector path dynamically with MoveIt:

```bash
ros2 launch mecharm_sim sim.launch.py use_moveit:=true
```

The pick-and-place node writes results to:

```text
~/.ros/mecharm_sim/pick_place_results.csv
```

Each successful run means the robot trajectory completed, the grasp assist command completed, and the ball pose was verified within `scene.placement_tolerance_m` of `scene.place_position`.

## Motion-Only Check

Use this when debugging controller startup without executing the pick-and-place sequence:

```bash
ros2 launch mecharm_sim sim.launch.py use_pick_place:=false
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /mnt/textop_big/mecharm/install/setup.bash
ros2 topic echo /joint_states
ros2 action list | grep follow_joint_trajectory
```

## Main Configuration

Task parameters live in `config/pick_place_poses.yaml`.

- `poses.home`, `poses.a_above`, `poses.a_pick`, `poses.b_above`, `poses.b_place`: six-axis arm poses in radians.
- `gripper.open`, `gripper.closed`: gripper controller positions.
- `scene.place_position`: target ball center position in Gazebo world coordinates.
- `scene.ball_position`: ball center position restored at the start of each attempt.
- `scene.reset_ball_each_attempt`: resets the detached ball to `scene.ball_position` before homing, making the first and repeated attempts deterministic.
- `scene.placement_tolerance_m`: maximum final ball position error accepted as success.
- `use_moveit`: when true, MoveIt plans Cartesian end-effector targets and `ros2_control` tracks every point in the returned `JointTrajectory`; the default false mode uses the calibrated joint waypoints.
- `scene.approach_height_m`: vertical clearance above the ball/target marker.
- `scene.grasp_height_offset_m`: gripper-base target height above the ball center for grasp; the default matches the calibrated fixed-pose geometry.
- `scene.place_height_offset_m`: gripper-base target height above the target center for release; the default matches the calibrated fixed-pose geometry.
- `scene.grasp_tolerance_m`: maximum allowed distance between the planned grasp target and the ball before the attach command is accepted.
- `scene.force_place_on_release`: optional deterministic placement fallback. The default is false so a bad pick/place pose fails instead of hiding the error.
- `use_detachable_joint`: when true, Gazebo's detachable joint system attaches the ball to the wrist during transport.

## Acceptance Test

For repeated validation, set `repeat_count: 5` in `config/pick_place_poses.yaml`, then run:

```bash
cd /mnt/textop_big/mecharm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mecharm_sim sim.launch.py
```

Expected result: at least 4 successful runs out of 5, no obvious table collision, and final ball placement within the configured tolerance.

## Troubleshooting

- If `ros2 launch` reports missing packages, install the dependencies listed in `docs/development.md`.
- If the arm aborts with `path tolerance violation`, increase trajectory duration slightly or relax the controller tolerances in `config/ros2_controllers.yaml`.
- If the launch fails with missing MoveIt packages, install the dependencies above first.
- If the log says `MoveIt failed to plan`, check that `mecharm_moveit_config` is built and that the target position is reachable.
- If the log says `Refusing to attach`, tune `scene.grasp_height_offset_m` and `scene.grasp_tolerance_m`, then confirm the gripper visually reaches the ball before closing.
- If the ball does not move with the gripper after a valid close-distance check, keep `use_detachable_joint: true` and confirm the attach/detach topics in `config/pick_place_poses.yaml` match the plugin in `urdf/mecharm_270_pick_place.urdf.xacro`.
- The Gazebo detachable-joint plugin starts attached. The task node deliberately sends `detach` and resets the ball before every attempt; do not remove this initialization step.
- If the terminal reports that the final placement error exceeds tolerance, check that `scene.place_position` uses the ball center height. In the default world, the table top is at `z=0.32`, the ball radius is `0.02`, and the ball center target is `z=0.345`.

See `docs/development.md` for the code logic chain, technology stack, package layout, and calibration notes.
