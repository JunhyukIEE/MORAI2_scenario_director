from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    scenario_share = get_package_share_directory('scenario_director')
    visualizer_share = get_package_share_directory('morai_visualizer')

    racing_params = [f"{scenario_share}/config/racing_director.yaml"]
    npc_params = [f"{scenario_share}/config/npc_controller.yaml"]

    return LaunchDescription([
        # Visualizer with correct topic mappings
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{visualizer_share}/launch/visualizer.launch.py"
            ),
            launch_arguments={
                'odom_topic': '/ego/odom',
                'opponent_odom_topic': '/opponent/odom',
                'opponent_odom_topic_2': '/npc2/odom',
                'opponent_odom_topic_3': '/npc3/odom',
            }.items(),
        ),

        # Racing Director (ego control with waypoint + overtake)
        Node(
            package='scenario_director',
            executable='racing_director_node',
            name='racing_director',
            parameters=racing_params,
            output='screen'
        ),

        # NPC Controller (opponent vehicle)
        Node(
            package='scenario_director',
            executable='npc_controller_node',
            name='npc_controller',
            parameters=npc_params,
            output='screen'
        ),
    ])
