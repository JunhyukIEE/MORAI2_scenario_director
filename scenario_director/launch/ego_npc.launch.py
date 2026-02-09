from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    visualizer_share = get_package_share_directory('morai_visualizer')
    ego_params = [
        f"{share}/config/scenario_director.yaml",
        f"{share}/config/pure_pursuit.yaml"
    ]
    local_planner_params = [
        f"{share}/config/scenario_director.yaml",
        f"{share}/config/local_path_planner.yaml"
    ]

    return LaunchDescription([
        # Visualizer (updated for new topic format)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{visualizer_share}/launch/visualizer.launch.py"
            ),
        ),
        # Local path planner
        Node(
            package='scenario_director',
            executable='local_path_planner_node',
            name='local_path_planner',
            parameters=local_planner_params,
            output='screen'
        ),
        # Ego vehicle controller (scenario director)
        Node(
            package='scenario_director',
            executable='scenario_director_node',
            name='scenario_director',
            parameters=ego_params,
            output='screen'
        ),
        # All 9 NPC controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{share}/launch/npc.launch.py"
            ),
        ),
    ])
