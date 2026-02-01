from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    udp_bridge_share = get_package_share_directory('udp_bridge')
    scenario_share = get_package_share_directory('scenario_director')

    udp_params = [f\"{udp_bridge_share}/config/udp_bridge.yaml\"]
    scenario_params = [f\"{scenario_share}/config/scenario_director.yaml\"]

    return LaunchDescription([
        Node(
            package='udp_bridge',
            executable='udp_receiver_node',
            name='udp_receiver',
            parameters=udp_params,
            output='screen'
        ),
        Node(
            package='udp_bridge',
            executable='udp_sender_node',
            name='udp_sender',
            parameters=udp_params,
            output='screen'
        ),
        Node(
            package='scenario_director',
            executable='scenario_director_node',
            name='scenario_director',
            parameters=scenario_params,
            output='screen'
        )
    ])
