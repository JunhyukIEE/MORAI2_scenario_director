from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('udp_bridge')
    params = [f"{share}/config/udp_bridge.yaml"]

    return LaunchDescription([
        Node(
            package='udp_bridge',
            executable='udp_receiver_node',
            name='udp_receiver',
            parameters=params,
            output='screen'
        ),
        Node(
            package='udp_bridge',
            executable='udp_sender_node',
            name='udp_sender',
            parameters=params,
            output='screen'
        ),
        Node(
            package='udp_bridge',
            executable='camera_receiver_node',
            name='camera_receiver',
            parameters=params,
            output='screen'
        )
    ])
