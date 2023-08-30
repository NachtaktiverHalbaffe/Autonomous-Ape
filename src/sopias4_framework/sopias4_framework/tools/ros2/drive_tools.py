import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def generate_twist_msg(direction: str, vel_rel: float = 1.0) -> Twist:
    drive_msg = Twist()

    if vel_rel > 1.0 or vel_rel < 0:
        raise ValueError("Wring velocity given. Make sure it's between 0 and 1")

    match direction:
        case "forward":
            drive_msg.linear.x = vel_rel
        case "backward":
            drive_msg.linear.x = -vel_rel
        case "left":
            drive_msg.angular.z = vel_rel
        case "right":
            drive_msg.angular.z = -vel_rel
        case "stop":
            # drive_msgs is already at 0 speed
            pass
        case _:
            raise ValueError(
                'Wrong direction given. Only "forward", "backward", "left", "right", "rotate_left" and "rotate_right" possible.'
            )

    return drive_msg


if __name__ == "__main__":
    print("This is only an library, dont execute this Python script at it's own")
