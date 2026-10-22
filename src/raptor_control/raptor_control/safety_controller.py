import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        # Safety parameters
        self.declare_parameter("stop_distance", 0.80)
        self.declare_parameter("scan_timeout", 0.50)
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("front_sector_deg", 20.0)

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.front_sector_deg = float(self.get_parameter("front_sector_deg").value)

        self.last_scan_time = None
        self.last_command_time = None

        self.front_distance = None
        self.scan_usable = False
        self.desired_command = Twist()

        self.command_subscriber = self.create_subscription(
            Twist,
            "/raptor/desired_cmd_vel",
            self.command_callback,
            10,
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            "/model/automated_car/scan",
            self.scan_callback,
            10,
        )

        self.safe_command_publisher = self.create_publisher(
            Twist,
            "/model/automated_car/cmd_vel",
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_callback,
        )

        self.last_status = None

        self.get_logger().info("Safety Controller started in FAIL-SAFE mode.")

    def command_callback(self, msg):
        self.desired_command = msg
        self.last_command_time = self.get_clock().now()

    def scan_callback(self, msg):
        distances = []
        usable_rays = 0
        total_rays = 0

        for i, distance in enumerate(msg.ranges):
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)

            if abs(angle) > self.front_sector_deg:
                continue

            total_rays += 1

            if math.isfinite(distance):
                if msg.range_min <= distance <= msg.range_max:
                    distances.append(distance)
                    usable_rays += 1

            elif math.isinf(distance) and distance > 0:
                # Gazebo LiDAR: no finite return in sensor range.
                usable_rays += 1

        self.scan_usable = total_rays > 0 and usable_rays == total_rays

        self.front_distance = min(distances) if distances else math.inf

        self.last_scan_time = self.get_clock().now()
        # self.get_logger().info(
        #     f"SCAN RECEIVED | "
        #     f"usable={self.scan_usable} | "
        #     f"front_distance={self.front_distance:.2f} m"
        # )

    def is_fresh(self, timestamp, timeout):
        if timestamp is None:
            return False

        age = (self.get_clock().now() - timestamp).nanoseconds / 1e9

        return 0.0 <= age <= timeout

    def control_callback(self):
        safe_command = Twist()
        status = "STOP"

        scan_fresh = self.is_fresh(
            self.last_scan_time,
            self.scan_timeout,
        )

        command_fresh = self.is_fresh(
            self.last_command_time,
            self.command_timeout,
        )

        if not scan_fresh:
            status = "STOP: LiDAR timeout"

        elif not self.scan_usable:
            status = "STOP: invalid LiDAR scan"

        elif not command_fresh:
            status = "STOP: command timeout"

        elif not (
            math.isfinite(self.desired_command.linear.x)
            and math.isfinite(self.desired_command.angular.z)
        ):
            status = "STOP: invalid command"

        elif (
            self.desired_command.linear.x > 0.0
            and self.front_distance <= self.stop_distance
        ):
            status = "STOP: obstacle ahead"

        else:
            safe_command.linear.x = self.desired_command.linear.x
            safe_command.angular.z = self.desired_command.angular.z
            status = "PASS: command allowed"

        self.safe_command_publisher.publish(safe_command)

        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_command_publisher.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
