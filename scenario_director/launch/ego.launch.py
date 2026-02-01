from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    params = [f"{share}/config/scenario_director.yaml"]

    return LaunchDescription([
        Node(
            package='scenario_director',
            executable='scenario_director_node',
            name='scenario_director',
            parameters=params,
            output='screen'
        )
    ])
