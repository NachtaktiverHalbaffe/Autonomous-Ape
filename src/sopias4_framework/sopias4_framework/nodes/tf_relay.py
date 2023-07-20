#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__("tf_relay")  # type: ignore
        tf_topic = f"{self.get_namespace()}/tf"
        self.frame_prefix = f"{self.get_namespace()}/"
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )
        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.publisher.publish(msg)


def main(args=None):
    """
    Start the node. It basically initializes the ROS2 context and creates a instance of TFRelay
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = TFRelay()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
