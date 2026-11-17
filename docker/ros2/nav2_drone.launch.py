import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": "/ros2_ws/nav2_drone_params.yaml",
        }.items(),
    )

    bridge = ExecuteProcess(
        cmd=["python3", "/ros2_ws/unity_nav2_px4_bridge.py"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/ros2_ws/drone_nav2.rviz"],
        output="screen",
    )

    return LaunchDescription([
        bridge,
        nav2_launch,
        rviz,
    ])

