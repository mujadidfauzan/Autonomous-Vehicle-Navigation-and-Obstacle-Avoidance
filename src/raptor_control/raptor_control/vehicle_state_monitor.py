import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class VehicleStateMonitor(Node):

    def __init__(self):
        super().__init__("vehicle_state_monitor")

        self.subscription = self.create_subscription(
            Odometry, "/model/automated_car/odometry", self.odometry_callback, 10
        )

        self.get_logger().info("Vehicle State Monitor started. Waiting for odometry...")

    def odometry_callback(self, msg):
        # Position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation quaternion
        q = msg.pose.pose.orientation

        # Quaternion -> Yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)

        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Velocity from odometry
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        self.get_logger().info(
            f"Position: ({x:.2f}, {y:.2f}) m | "
            f"Yaw: {math.degrees(yaw):.1f} deg | "
            f"Linear: {linear_velocity:.2f} m/s | "
            f"Angular: {angular_velocity:.2f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)

    node = VehicleStateMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
