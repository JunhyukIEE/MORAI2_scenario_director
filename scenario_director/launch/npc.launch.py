from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    params = [
        f"{share}/config/npc_controller.yaml",
        f"{share}/config/pure_pursuit.yaml"
    ]

    return LaunchDescription([
        Node(
            package='scenario_director',
            executable='npc_controller_node',
            name='npc_controller',
            parameters=params,
            output='screen'
        )
    ])
