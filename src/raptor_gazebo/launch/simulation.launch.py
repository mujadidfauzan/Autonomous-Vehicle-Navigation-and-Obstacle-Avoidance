import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    gazebo_share = get_package_share_directory("raptor_gazebo")
    description_share = get_package_share_directory("raptor_description")

    world_path = os.path.join(gazebo_share, "worlds", "raptor_world.sdf")

    model_path = os.path.join(description_share, "models", "raptor_car", "model.sdf")

    # 1. Start Gazebo Fortress
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "3", world_path],
        output="screen",
    )

    # 2. Spawn the Ackermann vehicle
    spawn_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "raptor_world",
            "-file",
            model_path,
            "-name",
            "automated_car",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # 3. ROS-Gazebo topic bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/automated_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/model/automated_car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/automated_car/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            bridge,
            TimerAction(
                period=5.0,
                actions=[spawn_vehicle],
            ),
        ]
    )
