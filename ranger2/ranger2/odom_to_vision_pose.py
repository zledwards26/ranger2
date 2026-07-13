#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped


class OdomToVisionPose(Node):
    def __init__(self):
        super().__init__('odom_to_vision_pose')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            qos,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            qos,
        )

        self.speed_pub = self.create_publisher(
            TwistStamped,
            '/mavros/vision_speed/speed_twist',
            qos,
        )

        self.count = 0
        self.get_logger().info(
            'odom_to_vision_pose bridge started: '
            '/zed/zed_node/odom -> /mavros/vision_pose/pose + '
            '/mavros/vision_speed/speed_twist'
        )

    def odom_callback(self, msg: Odometry):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header = msg.header
        twist_msg.header.frame_id = msg.child_frame_id
        twist_msg.twist = msg.twist.twist
        self.speed_pub.publish(twist_msg)

        self.count += 1
        if self.count % 150 == 0:
            self.get_logger().info(
                f'Relayed {self.count} pose+velocity messages to MAVROS'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomToVisionPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
