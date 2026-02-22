from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_name = "my_arm_description"
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, "urdf", "my_arm.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "demo_config.rviz")

    # URDF 파일 읽기
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    # IK Node
    ik_node = Node(
        package="my_arm_description",
        executable="my_arm_ik_node",
        output="screen"
    )

    # Sequential Controller Node
    seq_controller = Node(
        package="my_arm_description",
        executable="my_arm_seq_controller_node",
        output="screen"
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        ik_node,
        seq_controller,
        rviz
    ])