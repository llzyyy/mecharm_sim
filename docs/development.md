# mechArm 270 MoveIt Pick-and-Place Notes

## Target

The task is to make the simulated mechArm 270 move its gripper to the ball, close the gripper, carry the ball to the target marker, release it, and verify the final ball position.

The MoveIt control chain is:

1. Gazebo publishes the ball pose.
2. The task node builds end-effector target poses from the ball and target coordinates.
3. MoveIt plans an arm `JointTrajectory` for `gripper_base`.
4. The task node sends that planned trajectory to `arm_controller/follow_joint_trajectory`.
5. `ros2_control` executes the trajectory in Gazebo.
6. The gripper controller opens/closes the gripper.
7. Gazebo detachable joint attaches the ball only after the planned grasp pose is reached.
8. The task node verifies the final ball pose against `scene.place_position`.

MoveIt is used for planning. `ros2_control` remains responsible for executing the planned trajectory.

## Technology Stack

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim / Ignition Gazebo 6
- URDF/Xacro robot model
- MoveIt 2 `move_group`
- KDL kinematics plugin
- OMPL planner
- `ros2_control`
- `joint_trajectory_controller`
- ROS 2 Python `rclpy`
- Ignition Transport CLI for Gazebo pose readback and detachable-joint commands

## Packages

- `mecharm_sim`: Gazebo world, URDF, ROS controllers, launch file, and task node.
- `mecharm_moveit_config`: SRDF, kinematics, OMPL, joint limits, controller mapping, and `move_group` launch.
- `mycobot_description`: upstream mesh assets used by the mechArm URDF.

## Runtime Flow

`ros2 launch mecharm_sim sim.launch.py` starts:

1. Gazebo with `worlds/pick_place.sdf`.
2. `robot_state_publisher` with `urdf/mecharm_270_pick_place.urdf.xacro`.
3. The mechArm model in Gazebo.
4. `joint_state_broadcaster`.
5. `arm_controller`.
6. `gripper_action_controller`.
7. MoveIt's `move_group`.
8. `mecharm_sim/pick_place.py`.

`move_group` is started only when the launch argument `use_moveit:=true` is selected. The default mode follows the calibrated joint waypoints without starting MoveIt.

The URDF now puts `base` at `world z=0.34` through `world_to_base`. Gazebo spawns the model at `z=0.0`. This keeps MoveIt, TF, and Gazebo in the same world coordinate frame.

## Task Node Structure

`PickPlaceNode` owns the task state machine:

- `_prepare_robot()`: explicitly detach the initially-attached Gazebo joint, restore the ball source pose, open the gripper, and move to home.
- `_pick_ball()`: read ball pose, use MoveIt to plan approach/grasp, execute with ros2_control, close gripper, validate grasp proximity, attach.
- `_place_ball()`: use MoveIt to plan target approach/release, execute with ros2_control, open gripper, detach, verify final ball pose.
- `_move_gripper()`: send gripper `FollowJointTrajectory`.
- `_execute_planned_arm_trajectory()`: time-scale and track every point in MoveIt's planned arm trajectory through `arm_controller`.

`MoveItPosePlanner` is intentionally small:

- Wait for `/plan_kinematic_path`.
- Build a MoveIt `MotionPlanRequest`.
- Request a plan for `gripper_base`.
- Return the planned `JointTrajectory`.

It does not execute the trajectory itself.

## Important Parameters

`config/pick_place_poses.yaml`:

- `use_moveit`: enables MoveIt planning mode.
- `scene.place_position`: target ball center position.
- `scene.ball_position`: source ball center position restored before each attempt.
- `scene.reset_ball_each_attempt`: detaches and restores the ball before homing so the plugin's initially-attached state cannot lock the arm.
- `scene.approach_height_m`: clearance above ball/target.
- `scene.grasp_height_offset_m`: gripper-base target height above ball center.
- `scene.place_height_offset_m`: gripper-base target height above target center.
- `scene.grasp_tolerance_m`: attach is refused if the planned grasp target is too far from the ball.
- `scene.placement_tolerance_m`: final placement acceptance radius.
- `scene.force_place_on_release`: optional deterministic fallback; default false.
- `moveit.group_name`: MoveIt planning group, default `arm`.
- `moveit.end_effector_link`: default `gripper_base`.

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
cd /mnt/textop_big/mecharm
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mecharm_sim sim.launch.py
```

Enable dynamic MoveIt pose planning with:

```bash
ros2 launch mecharm_sim sim.launch.py use_moveit:=true
```

For controller-only debugging:

```bash
ros2 launch mecharm_sim sim.launch.py use_pick_place:=false use_moveit:=false
```

## Verification

```bash
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep plan_kinematic_path
ign topic -t /world/mecharm_pick_place/pose/info -e -n 1 --json-output
tail -n 20 ~/.ros/mecharm_sim/pick_place_results.csv
```

Success requires both controller execution success and final ball placement within `scene.placement_tolerance_m`.
