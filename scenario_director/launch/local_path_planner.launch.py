from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    params = [f"{share}/config/local_path_planner.yaml"]

    return LaunchDescription([
        Node(
            package='scenario_director',
            executable='local_path_planner_node',
            name='local_path_planner',
            parameters=params,
            output='screen'
        )
    ])
