# Experiment 3 Fake Vision: Ubuntu Validation Checklist

This Windows pass validates source structure and static syntax only. It does **not**
claim that ROS 2, Gazebo, MoveIt, ros2_control, or DetachableJoint ran successfully.

## 1. Dependencies and build

- [ ] Use Ubuntu with ROS 2 Humble and source `/opt/ros/humble/setup.bash`.
- [ ] Install/confirm `ros-humble-moveit`,
  `ros-humble-moveit-ros-move-group`,
  `ros-humble-moveit-planners-ompl`,
  `ros-humble-moveit-simple-controller-manager`,
  `ros-humble-ros-gz-sim`, `ros-humble-gz-ros2-control`,
  `ros-humble-ros2-control`, `ros-humble-ros2-controllers`,
  `ros-humble-control-msgs`, `ros-humble-xacro`,
  `ros-humble-robot-state-publisher`, and `python3-yaml`.
- [ ] Confirm sibling packages `mycobot_description` and
  `mecharm_moveit_config` are present in the workspace.
- [ ] Run:

  ```bash
  colcon build --symlink-install --packages-select \
    mycobot_description mecharm_moveit_config mecharm_sim
  source install/setup.bash
  ```

- [ ] Run `colcon test --packages-select mecharm_sim` and inspect
  `colcon test-result --verbose`.
- [ ] Confirm all four executables remain installed:
  `pick_place`, `fake_vision`, `classification_manager`, and
  `classification_pick_place`.

## 2. Static ROS/Gazebo parsing

- [ ] Run `xacro urdf/mecharm_270_classification.urdf.xacro` and inspect the
  generated URDF for six gripper DetachableJoint plugins.
- [ ] Run the Gazebo/SDF validation command available in the installed Gazebo
  version against `worlds/classification_fake.sdf`.
- [ ] Check whether this Humble installation expects `ign` or `gz` CLI and
  plugin naming. Keep the original pick-ball version unchanged if compatibility
  edits are required for the classification files.
- [ ] Confirm `classification_fake.launch.py --show-args` resolves the world,
  Xacro, controller YAML, and classification YAML from the installed share path.

## 3. Launch and controllers

- [ ] First launch without task execution:

  ```bash
  ros2 launch mecharm_sim classification_fake.launch.py \
    use_classification:=false use_moveit:=false
  ```

- [ ] Confirm Gazebo loads the table, six grid markers, six dynamic objects, two
  bin markers, and one mechArm.
- [ ] Confirm `/clock`, `/joint_states`, `/controller_manager/list_controllers`,
  `/arm_controller/commands`, and the gripper trajectory action are available.
- [ ] Confirm `joint_state_broadcaster`, `arm_controller`, and
  `gripper_action_controller` become active.
- [ ] Verify the robot base height (`robot_base_z=0.34`) and MoveIt world/base
  frames match the Gazebo robot pose.

## 4. DetachableJoint validation

- [ ] Verify Gazebo permits six DetachableJoint instances on robot `link6` and
  six instances on `table::table_link`.
- [ ] Confirm every child model uses child link `object_link`.
- [ ] Confirm all six gripper joints start attached as expected by the plugin and
  are detached by `classification_pick_place.initialize()`.
- [ ] Confirm all six table locks start attached and hold inactive objects fixed.
- [ ] For each model, manually exercise its four configured attach/detach topics
  and observe the corresponding state topic.
- [ ] Verify ownership transfer order: close gripper, attach gripper joint, then
  detach table lock. Verify release order: open gripper, then detach gripper joint.
- [ ] If multiple plugin instances conflict, implement an equivalent
  classification-only multi-target binding mechanism; do not modify the original
  pick-ball Xacro/world.

## 5. Fake Vision and manager topics

- [ ] Launch `fake_vision` alone and run
  `ros2 topic echo /classification/detections std_msgs/msg/String`.
- [ ] Confirm every frame has six detections and each has `model_name`,
  `class_name`, `grid_id`, and `confidence`, with no bin assignment.
- [ ] Confirm Manager alone maps cube to `cube_bin` and cylinder to
  `cylinder_bin` on `/classification/command`.
- [ ] Inspect `/classification/result` for `pick`, `place`, `return`, and
  final success/failure events with the same `request_id`.
- [ ] Inject test JSON for missing grids, unsupported classes, low confidence,
  unreachable grids, malformed commands, timeouts, and pick failures. Confirm
  `empty`, `unknown`, `unreachable`, and `pick_failed` logging.
- [ ] Confirm repeated Fake Vision frames never cause a processed grid to run
  twice during one Manager process.

## 6. Coordinate calibration and reachability

- [ ] Measure the Gazebo pose of all six objects and compare it with
  `grids.*.world_position` and `objects[*].initial_position`.
- [ ] Calibrate all six grid XY values and the shared object-center Z. The current
  40 mm objects assume table top `z=0.32` and center `z=0.34`.
- [ ] Calibrate `cube_bin.world_position`, `cylinder_bin.world_position`, and
  all three drop offsets per bin.
- [ ] Check MoveIt reachability and collision-free approach/lift paths for every
  grid and every bin slot before enabling the full task.
- [ ] Tune approach height, gripper center offset, tolerances, velocity scaling,
  and placement corrections only in `classification_fake.yaml`.
- [ ] Keep `use_moveit:=true` initially. The inherited joint-space fallback
  poses are not calibrated for all six grids.

## 7. Incremental and full experiment

- [ ] Test one object at a time by temporarily publishing a controlled detection
  frame; verify model pose before pick, after attach, before release, and after
  placement.
- [ ] Test one cube and one cylinder, then three objects, then all six.
- [ ] Run:

  ```bash
  ros2 launch mecharm_sim classification_fake.launch.py use_moveit:=true
  ```

- [ ] Verify exactly three cubes finish in the three `cube_bin` slots and exactly
  three cylinders finish in the three `cylinder_bin` slots.
- [ ] Verify no inactive object moves, no target is processed twice, all six
  requests return home, and final pose error stays within
  `scene.placement_tolerance_m`.
- [ ] Review `~/.ros/mecharm_sim/classification_results.csv` and
  `~/.ros/mecharm_sim/classification_manager.jsonl`.
- [ ] Re-run the original `ros2 launch mecharm_sim sim.launch.py` acceptance
  test to prove the independent pick-ball version still works unchanged.
