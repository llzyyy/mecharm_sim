from robomaster import robot
import socket
import time

COMMAND_PORT = 5006
STATUS_ADDRESS = ("127.0.0.1", 5005)

status_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

command_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

command_socket.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

command_socket.bind(("0.0.0.0", COMMAND_PORT))
command_socket.settimeout(1.0)


def send_status(message):
    print("[STATUS]", message)

    status_socket.sendto(
        message.encode("utf-8"),
        STATUS_ADDRESS
    )


ep_robot = robot.Robot()
chassis = None

try:
    print("Connecting to RoboMaster...")

    ep_robot.initialize(conn_type="ap")

    arm = ep_robot.robotic_arm
    gripper = ep_robot.gripper
    chassis = ep_robot.chassis

    print("RoboMaster connected")
    print("Waiting for ROS2 commands...")

    while True:

        try:
            data, address = command_socket.recvfrom(1024)

        except socket.timeout:
            send_status("READY")
            continue

        command = data.decode("utf-8").strip()

        print("[ROS2 COMMAND]", command)

        try:

            if command == "OPEN_GRIPPER":

                gripper.open()
                time.sleep(1)

            elif command == "ARM_DOWN":

                arm.move(
                    x=0,
                    y=-100
                ).wait_for_completed()

                time.sleep(1)

            elif command == "CLOSE_GRIPPER":

                gripper.close()
                time.sleep(0.5)

            elif command == "ARM_UP":

                arm.move(
                    x=0,
                    y=100
                ).wait_for_completed()

                time.sleep(1.5)

            elif command == "ROTATE":

                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=120
                )

                time.sleep(0.6)

                chassis.drive_speed(
                    x=0,
                    y=0,
                    z=0
                )

                time.sleep(1)

            else:

                send_status(
                    "ERROR:UNKNOWN_COMMAND:" + command
                )

                continue

            send_status(
                "DONE:" + command
            )

        except Exception as e:

            send_status(
                "ERROR:" + command + ":" + str(e)
            )

except KeyboardInterrupt:

    print("Stopped by user")

finally:

    if chassis is not None:

        try:
            chassis.drive_speed(
                x=0,
                y=0,
                z=0
            )
        except Exception:
            pass

    try:
        ep_robot.close()
    except Exception:
        pass

    command_socket.close()
    status_socket.close()

    print("RoboMaster connection closed")