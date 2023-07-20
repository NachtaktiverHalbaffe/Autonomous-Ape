#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFStaticRelay(Node):
    def __init__(self):
        super().__init__("tf_static_relay")  # type: ignore
        tf_static_topic = f"{self.get_namespace()}/tf_static"
        self.frame_prefix = f"{self.get_namespace()}/"
        self.static_subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_static_topic,
            callback=self.static_tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

        self.static_publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf_static",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

    def static_tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.static_publisher.publish(msg)


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of TFStaticRelay
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = TFStaticRelay()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
