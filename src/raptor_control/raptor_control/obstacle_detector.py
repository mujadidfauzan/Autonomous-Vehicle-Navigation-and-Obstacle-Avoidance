import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        self.scan_subscriber = self.create_subscription(
            LaserScan, "/model/automated_car/scan", self.scan_callback, 10
        )

        self.last_log_time = None

        self.get_logger().info("Obstacle Detector started. Waiting for LiDAR data...")

    def get_min_distance(self, scan, min_angle, max_angle):
        """
        Return minimum valid distance within an angular sector.

        Angles are specified in degrees.
        Returns None when no valid finite measurement exists.
        """
        valid_distances = []

        for i, distance in enumerate(scan.ranges):
            angle_rad = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle_rad)

            if not (min_angle <= angle_deg <= max_angle):
                continue

            if not math.isfinite(distance):
                continue

            if not (scan.range_min <= distance <= scan.range_max):
                continue

            valid_distances.append(distance)

        if not valid_distances:
            return None

        return min(valid_distances)

    def scan_callback(self, msg):
        front = self.get_min_distance(msg, -20.0, 20.0)
        left = self.get_min_distance(msg, 20.0, 90.0)
        right = self.get_min_distance(msg, -90.0, -20.0)

        current_time = self.get_clock().now()

        if self.last_log_time is not None:
            elapsed = (current_time - self.last_log_time).nanoseconds / 1e9

            if elapsed < 1.0:
                return

        self.last_log_time = current_time

        def format_distance(value):
            if value is None:
                return "N/A"
            return f"{value:.2f} m"

        self.get_logger().info(
            f"Front: {format_distance(front)} | "
            f"Left: {format_distance(left)} | "
            f"Right: {format_distance(right)}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
