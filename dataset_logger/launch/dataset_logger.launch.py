from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('dataset_logger')
    params = [f"{share}/config/dataset_logger.yaml"]

    return LaunchDescription([
        Node(
            package='dataset_logger',
            executable='dataset_logger_node',
            name='dataset_logger',
            parameters=params,
            output='screen'
        )
    ])
