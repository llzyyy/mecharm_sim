# RoboMaster Real-Robot Verification

This folder contains the real-robot control system used for the fixed-point grasping experiment.

The RoboMaster platform is controlled by ROS 2 through a small communication bridge between WSL and Windows.

## System Architecture

The control flow is:

ROS 2 Experiment Controller  
→ `/robomaster/command`  
→ ROS 2 Bridge  
→ UDP  
→ Windows Hardware Adapter  
→ RoboMaster SDK  
→ RoboMaster

Robot feedback is returned through:

RoboMaster  
→ Hardware Adapter  
→ UDP  
→ ROS 2 Bridge  
→ ROS 2 topics

## Main Files

- `ros2/robomaster_experiment_controller.py`  
  Runs one fixed-point grasping and placing sequence.

- `ros2/robomaster_ros2_bridge.py`  
  Transfers commands and robot feedback between ROS 2 and Windows.

- `windows/robomaster_hardware_adapter.py`  
  Uses the RoboMaster Python SDK to execute the real robot actions.

- `config/robomaster_params.yaml`  
  Stores motion, communication and timeout parameters.

- `launch/robomaster_real_robot.launch.py`  
  Starts the ROS 2 controller and bridge together.

## Fixed-Point Grasping Sequence

The experiment uses the following action sequence:

1. Open gripper
2. Move arm down
3. Close gripper
4. Lift arm
5. Rotate toward the placing position
6. Move arm down
7. Open gripper
8. Lift arm to the safe position

The object is placed at a fixed grasping position, so visual detection is not required.

## ROS 2 Topics

### `/robomaster/command`

Used by the experiment controller to send commands such as:

- `OPEN_GRIPPER`
- `ARM_DOWN`
- `CLOSE_GRIPPER`
- `ARM_UP`
- `ROTATE`

### `/robomaster/status`

Used for task-level feedback:

- `READY`
- `DONE:<COMMAND>`
- `ERROR:<MESSAGE>`

The controller waits for the correct `DONE` message before starting the next action.

### `/robomaster/arm_position`

Publishes the real RoboMaster arm position during operation.

The Windows adapter obtains the arm position from the RoboMaster SDK and the ROS 2 bridge publishes it as a `geometry_msgs/Point` message.

## Safety and Error Handling

Each command has a timeout configured in the YAML file.

If the robot reports an error or a command does not finish within the timeout, the experiment controller stops the current cycle.

The chassis is also stopped when the adapter encounters an execution error or when the program exits.

## Configuration

The main experiment parameters are stored in:

```text
config/robomaster_params.yaml
