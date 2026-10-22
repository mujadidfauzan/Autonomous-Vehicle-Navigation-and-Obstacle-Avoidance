import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


class WaypointController(Node):

    def __init__(self):
        super().__init__("waypoint_controller")

        # Vehicle configuration
        self.wheelbase = 0.30
        self.max_steering = 0.60

        # Navigation target
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value

        # Controller configuration
        self.target_speed = 0.30
        self.goal_tolerance = 0.20
        self.minimum_lookahead = 0.50
        self.control_period = 0.10

        # Vehicle state
        self.x = None
        self.y = None
        self.yaw = None
        self.last_odom_time = None

        self.goal_reached = False
        self.last_log_time = 0.0

        self.cmd_publisher = self.create_publisher(Twist, "/raptor/desired_cmd_vel", 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, "/model/automated_car/odometry", self.odometry_callback, 10
        )

        self.timer = self.create_timer(self.control_period, self.control_callback)

        self.get_logger().info(
            f"Waypoint Controller started. "
            f"Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y**2 + q.z**2)

        self.yaw = math.atan2(siny, cosy)
        self.last_odom_time = self.get_clock().now()

    def publish_stop(self):
        self.cmd_publisher.publish(Twist())

    def control_callback(self):
        # Wait until odometry is available
        if self.x is None or self.last_odom_time is None:
            self.publish_stop()
            return

        # Stop if odometry becomes stale
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9

        if age > 0.5:
            self.publish_stop()
            self.get_logger().warn(
                "Odometry timeout. Vehicle stopped.", throttle_duration_sec=2.0
            )
            return

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.publish_stop()

            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    f"GOAL REACHED! Final distance: " f"{distance:.3f} m"
                )
            return

        # Target direction and heading error
        target_heading = math.atan2(dy, dx)
        alpha = wrap_angle(target_heading - self.yaw)

        # Goal-point curvature (Pure Pursuit geometry)
        lookahead = max(distance, self.minimum_lookahead)
        curvature = 2.0 * math.sin(alpha) / lookahead

        # Convert curvature into steering angle
        steering = math.atan(self.wheelbase * curvature)

        # Respect the steering limit
        steering = clamp(steering, -self.max_steering, self.max_steering)

        # Convert steering back to achievable curvature
        curvature = math.tan(steering) / self.wheelbase

        # Reduce speed when approaching goal
        speed = min(self.target_speed, 0.8 * distance)

        # Twist angular.z is yaw rate, not steering angle
        yaw_rate = speed * curvature

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = yaw_rate

        self.cmd_publisher.publish(cmd)

        # Print diagnostics approximately once per second
        now_seconds = self.get_clock().now().nanoseconds / 1e9

        if now_seconds - self.last_log_time >= 1.0:
            self.last_log_time = now_seconds

            self.get_logger().info(
                f"Position=({self.x:.2f}, {self.y:.2f}) | "
                f"Distance={distance:.2f} m | "
                f"Heading error={math.degrees(alpha):.1f} deg | "
                f"Steering={math.degrees(steering):.1f} deg | "
                f"Speed={speed:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
