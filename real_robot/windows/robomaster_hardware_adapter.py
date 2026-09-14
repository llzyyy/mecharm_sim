from pathlib import Path
from robomaster import robot

import socket
import time
import yaml


# Read the configuration file from the same folder as this program.
config_path = (
    Path(__file__).resolve().parent
    / "robomaster_params.yaml"
)

with open(config_path, "r", encoding="utf-8") as file:
    config = yaml.safe_load(file)

bridge_config = config["bridge"]
robot_config = config["robot"]


# Network parameters used by the ROS 2 bridge.
COMMAND_PORT = int(
    bridge_config["command_port"]
)

STATUS_PORT = int(
    bridge_config["status_port"]
)

STATUS_ADDRESS = (
    bridge_config["windows_address"],
    STATUS_PORT
)


# Robot motion parameters.
ARM_DOWN_Y = int(
    robot_config["arm_down_y"]
)

ARM_UP_Y = int(
    robot_config["arm_up_y"]
)

ROTATE_SPEED_Z = float(
    robot_config["rotate_speed_z"]
)

ROTATE_DURATION = float(
    robot_config["rotate_duration"]
)

GRIPPER_OPEN_WAIT = float(
    robot_config["gripper_open_wait"]
)

GRIPPER_CLOSE_WAIT = float(
    robot_config["gripper_close_wait"]
)

ARM_DOWN_WAIT = float(
    robot_config["arm_down_wait"]
)

ARM_UP_WAIT = float(
    robot_config["arm_up_wait"]
)

ROTATE_STOP_WAIT = float(
    robot_config["rotate_stop_wait"]
)

ARM_POSITION_FREQUENCY = int(
    robot_config["arm_position_frequency"]
)


# This socket sends robot status back to the ROS 2 bridge.
status_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# This socket receives commands from the ROS 2 bridge.
command_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

command_socket.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

command_socket.bind(
    ("0.0.0.0", COMMAND_PORT)
)

# A short timeout allows the adapter to keep reporting READY
# while it is waiting for the next ROS 2 command.
command_socket.settimeout(1.0)


def send_status(message):
    # Send task status or arm feedback to ROS 2.
    print("[STATUS]", message)

    status_socket.sendto(
        message.encode("utf-8"),
        STATUS_ADDRESS
    )


def arm_position_callback(position_info):
    # RoboMaster provides the current arm x and y position here.
    # The values are forwarded to ROS 2 for live monitoring.
    try:
        pos_x, pos_y = position_info

        send_status(
            f"ARM_POSITION:{pos_x},{pos_y}"
        )

    except Exception as e:
        print(
            "[POSITION CALLBACK ERROR]",
            e
        )


# Create the RoboMaster object first.
# The hardware modules are available after connection succeeds.
ep_robot = robot.Robot()

arm = None
gripper = None
chassis = None


try:
    print(
        "Loaded configuration:",
        config_path
    )

    print(
        f"Command port: {COMMAND_PORT}"
    )

    print(
        f"Status destination: "
        f"{STATUS_ADDRESS[0]}:{STATUS_ADDRESS[1]}"
    )

    print(
        f"Arm down/up: "
        f"{ARM_DOWN_Y} / {ARM_UP_Y}"
    )

    print(
        f"Rotation: {ROTATE_SPEED_Z} deg/s "
        f"for {ROTATE_DURATION} s"
    )

    print("Connecting to RoboMaster...")

    # Connect to the real robot using RoboMaster AP mode.
    ep_robot.initialize(
        conn_type="ap"
    )

    arm = ep_robot.robotic_arm
    gripper = ep_robot.gripper
    chassis = ep_robot.chassis

    # Send the real arm position back to ROS 2 several times per second.
    arm.sub_position(
        freq=ARM_POSITION_FREQUENCY,
        callback=arm_position_callback
    )

    print("RoboMaster connected")

    print(
        f"Arm position feedback enabled at "
        f"{ARM_POSITION_FREQUENCY} Hz"
    )

    print("Waiting for ROS2 commands...")


    while True:

        try:
            # Commands arrive from the ROS 2 bridge through UDP.
            data, address = command_socket.recvfrom(
                1024
            )

        except socket.timeout:
            # READY tells the controller that the adapter is alive
            # and ready to start or continue an experiment.
            send_status("READY")
            continue


        command = (
            data.decode("utf-8")
            .strip()
        )

        print(
            "[ROS2 COMMAND]",
            command
        )


        try:

            # Open the gripper before approaching the object.
            if command == "OPEN_GRIPPER":

                gripper.open()

                time.sleep(
                    GRIPPER_OPEN_WAIT
                )


            # Move downward to the fixed grasping position.
            elif command == "ARM_DOWN":

                arm.move(
                    x=0,
                    y=ARM_DOWN_Y
                ).wait_for_completed()

                time.sleep(
                    ARM_DOWN_WAIT
                )


            # Close the gripper around the object.
            elif command == "CLOSE_GRIPPER":

                gripper.close()

                time.sleep(
                    GRIPPER_CLOSE_WAIT
                )


            # Lift the arm to the safe transport height.
            elif command == "ARM_UP":

                arm.move(
                    x=0,
                    y=ARM_UP_Y
                ).wait_for_completed()

                time.sleep(
                    ARM_UP_WAIT
                )


            # Rotate toward the fixed placing direction.
            elif command == "ROTATE":

                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=ROTATE_SPEED_Z
                )

                time.sleep(
                    ROTATE_DURATION
                )

                # Stop the chassis after the required rotation.
                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=0
                )

                time.sleep(
                    ROTATE_STOP_WAIT
                )


            # Any command outside the experiment sequence is rejected.
            else:

                send_status(
                    "ERROR:UNKNOWN_COMMAND:"
                    + command
                )

                continue


            # DONE allows the ROS 2 controller to move to the next step.
            send_status(
                "DONE:" + command
            )


        except Exception as e:

            # Stop chassis motion first if something fails.
            try:
                if chassis is not None:

                    chassis.drive_speed(
                        x=0,
                        y=0,
                        z=0
                    )

            except Exception:
                pass

            # Report the failure so the ROS 2 controller stops the cycle.
            send_status(
                "ERROR:"
                + command
                + ":"
                + str(e)
            )


except KeyboardInterrupt:
    # Ctrl+C can still be used for a manual stop.
    print("Stopped by user")


finally:
    print(
        "Cleaning up RoboMaster connection..."
    )

    # Make sure chassis motion has stopped before leaving the program.
    if chassis is not None:

        try:
            chassis.drive_speed(
                x=0,
                y=0,
                z=0
            )

        except Exception:
            pass


    # Stop the live arm position subscription.
    if arm is not None:

        try:
            arm.unsub_position()

        except Exception:
            pass


    # Close the connection to the physical robot.
    try:
        ep_robot.close()

    except Exception:
        pass


    # Close both UDP sockets.
    try:
        command_socket.close()

    except Exception:
        pass

    try:
        status_socket.close()

    except Exception:
        pass


    print(
        "RoboMaster connection closed"
    )