from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

from ariac_moveit_config.parameters import generate_parameters


def generate_launch_description():
    # Robot Commander Node
    robot_control = Node(
        package="ariac_tutorials",
        executable="robot_control",
        output="screen",
        parameters=generate_parameters()
    )

    # test = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ariac_tutorials"), "/scripts", "/tutorial.py"]
    #     )
    # )

    # # MoveIt node
    # moveit = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
    #     )
    # )

    return LaunchDescription([robot_control])
