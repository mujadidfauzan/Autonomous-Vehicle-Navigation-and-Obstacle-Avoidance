import math

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from raptor_control.safety_controller import SafetyController


def make_scan(distance):
    msg = LaserScan()

    msg.angle_min = math.radians(-20.0)
    msg.angle_increment = math.radians(1.0)
    msg.angle_max = math.radians(20.0)

    msg.range_min = 0.10
    msg.range_max = 10.0

    msg.ranges = [distance] * 41

    return msg


def main():
    rclpy.init()
    node = SafetyController()

    cases = [
        ("Valid obstacle", 0.50, True, 0.50),
        ("No return", math.inf, True, math.inf),
        ("NaN", math.nan, False, math.inf),
        ("Negative infinity", -math.inf, False, math.inf),
        ("Below minimum", 0.05, False, math.inf),
        ("Above maximum", 12.0, False, math.inf),
    ]

    try:
        for name, value, expected_usable, expected_distance in cases:
            node.scan_callback(make_scan(value))

            usable_ok = node.scan_usable == expected_usable
            distance_ok = node.front_distance == expected_distance

            passed = usable_ok and distance_ok

            print(
                f"{name}: "
                f"{'PASS' if passed else 'FAIL'} | "
                f"usable={node.scan_usable} | "
                f"distance={node.front_distance}"
            )

        print("\n--- FAIL-SAFE OUTPUT TEST ---")

        # Simulate a fresh forward command.
        cmd = Twist()
        cmd.linear.x = 0.10
        node.command_callback(cmd)

        # Simulate an invalid LiDAR scan.
        node.scan_callback(make_scan(math.nan))

        # Capture the command published by control_callback.
        published_commands = []

        original_publish = node.safe_command_publisher.publish

        try:
            node.safe_command_publisher.publish = lambda msg: published_commands.append(
                msg
            )

            node.control_callback()

        finally:
            node.safe_command_publisher.publish = original_publish

        if not published_commands:
            raise AssertionError("Safety Controller published no command")

        output = published_commands[-1]

        passed = output.linear.x == 0.0 and output.angular.z == 0.0

        print(
            f"Invalid LiDAR -> STOP: "
            f"{'PASS' if passed else 'FAIL'} | "
            f"linear.x={output.linear.x:.2f} | "
            f"angular.z={output.angular.z:.2f}"
        )

        if not passed:
            raise AssertionError("Invalid LiDAR did not trigger STOP")

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
