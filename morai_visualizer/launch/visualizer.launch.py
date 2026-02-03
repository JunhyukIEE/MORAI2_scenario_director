from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    frame_id = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_vehicle_marker = LaunchConfiguration('use_vehicle_marker')
    use_opponent_marker = LaunchConfiguration('use_opponent_marker')
    use_waypoints_path = LaunchConfiguration('use_waypoints_path')
    use_static_map_odom_tf = LaunchConfiguration('use_static_map_odom_tf')
    use_static_world_map_tf = LaunchConfiguration('use_static_world_map_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    opponent_odom_topic = LaunchConfiguration('opponent_odom_topic')
    opponent_odom_topic_2 = LaunchConfiguration('opponent_odom_topic_2')
    opponent_odom_topic_3 = LaunchConfiguration('opponent_odom_topic_3')
    waypoints_csv = LaunchConfiguration('waypoints_csv')
    box_length = LaunchConfiguration('box_length')
    box_width = LaunchConfiguration('box_width')
    box_height = LaunchConfiguration('box_height')
    use_msg_frame = LaunchConfiguration('use_msg_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('scenario_director'),
                'map',
                'Sangam_map.yaml',
            ]),
            description='Path to map YAML for nav2_map_server',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='RViz fixed frame id',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('morai_visualizer'),
                'rviz',
                'morai_visualizer.rviz',
            ]),
            description='Path to RViz2 config',
        ),
        DeclareLaunchArgument(
            'use_vehicle_marker',
            default_value='true',
            description='Publish vehicle box marker from /odom',
        ),
        DeclareLaunchArgument(
            'use_opponent_marker',
            default_value='true',
            description='Publish opponent box marker from /opponent/odom',
        ),
        DeclareLaunchArgument(
            'use_waypoints_path',
            default_value='true',
            description='Publish waypoints path for RViz',
        ),
        DeclareLaunchArgument(
            'use_static_map_odom_tf',
            default_value='true',
            description='Publish a static map->odom transform at 0,0,0,0,0,0',
        ),
        DeclareLaunchArgument(
            'use_static_world_map_tf',
            default_value='true',
            description='Publish a static world->map transform at 0,0,0,0,0,0',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/vehicle1/odom',
            description='Odometry topic for vehicle pose',
        ),
        DeclareLaunchArgument(
            'opponent_odom_topic',
            default_value='/vehicle2/odom',
            description='Odometry topic for opponent pose',
        ),
        DeclareLaunchArgument(
            'opponent_odom_topic_2',
            default_value='/vehicle3/odom',
            description='Odometry topic for opponent pose (vehicle3)',
        ),
        DeclareLaunchArgument(
            'opponent_odom_topic_3',
            default_value='/vehicle4/odom',
            description='Odometry topic for opponent pose (vehicle4)',
        ),
        DeclareLaunchArgument(
            'waypoints_csv',
            default_value=PathJoinSubstitution([
                FindPackageShare('scenario_director'),
                'map',
                'waypoints.csv',
            ]),
            description='Waypoints CSV path',
        ),
        DeclareLaunchArgument(
            'box_length',
            default_value='4.0',
            description='Vehicle box length in meters',
        ),
        DeclareLaunchArgument(
            'box_width',
            default_value='1.8',
            description='Vehicle box width in meters',
        ),
        DeclareLaunchArgument(
            'box_height',
            default_value='1.0',
            description='Vehicle box height in meters',
        ),
        DeclareLaunchArgument(
            'use_msg_frame',
            default_value='true',
            description='Use frame_id from incoming odometry messages',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': map_yaml,
                'frame_id': frame_id,
            }],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='map_server_lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
            condition=IfCondition(use_static_map_odom_tf),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen',
            condition=IfCondition(use_static_world_map_tf),
        ),
        Node(
            package='morai_visualizer',
            executable='waypoints_path',
            name='waypoints_path',
            parameters=[{
                'waypoints_csv': waypoints_csv,
                'frame_id': frame_id,
                'topic': '/waypoints_path',
                'publish_rate_hz': 1.0,
            }],
            output='screen',
            condition=IfCondition(use_waypoints_path),
        ),
        Node(
            package='morai_visualizer',
            executable='vehicle_marker',
            name='vehicle_marker',
            parameters=[{
                'odom_topic': odom_topic,
                'frame_id': frame_id,
                'use_msg_frame': ParameterValue(use_msg_frame, value_type=bool),
                'box_length': ParameterValue(box_length, value_type=float),
                'box_width': ParameterValue(box_width, value_type=float),
                'box_height': ParameterValue(box_height, value_type=float),
                'marker_ns': 'ego_vehicle',
                'marker_id': 0,
            }],
            output='screen',
            condition=IfCondition(use_vehicle_marker),
        ),
        Node(
            package='morai_visualizer',
            executable='vehicle_marker',
            name='opponent_marker',
            parameters=[{
                'odom_topic': opponent_odom_topic,
                'frame_id': frame_id,
                'use_msg_frame': ParameterValue(use_msg_frame, value_type=bool),
                'box_length': ParameterValue(box_length, value_type=float),
                'box_width': ParameterValue(box_width, value_type=float),
                'box_height': ParameterValue(box_height, value_type=float),
                'marker_ns': 'opponent_vehicle',
                'marker_id': 1,
                'color_rgba': [0.2, 0.6, 1.0, 0.9],
            }],
            output='screen',
            condition=IfCondition(use_opponent_marker),
        ),
        Node(
            package='morai_visualizer',
            executable='vehicle_marker',
            name='opponent_marker_2',
            parameters=[{
                'odom_topic': opponent_odom_topic_2,
                'frame_id': frame_id,
                'use_msg_frame': ParameterValue(use_msg_frame, value_type=bool),
                'box_length': ParameterValue(box_length, value_type=float),
                'box_width': ParameterValue(box_width, value_type=float),
                'box_height': ParameterValue(box_height, value_type=float),
                'marker_ns': 'opponent_vehicle_2',
                'marker_id': 2,
                'color_rgba': [1.0, 0.6, 0.2, 0.9],
            }],
            output='screen',
            condition=IfCondition(use_opponent_marker),
        ),
        Node(
            package='morai_visualizer',
            executable='vehicle_marker',
            name='opponent_marker_3',
            parameters=[{
                'odom_topic': opponent_odom_topic_3,
                'frame_id': frame_id,
                'use_msg_frame': ParameterValue(use_msg_frame, value_type=bool),
                'box_length': ParameterValue(box_length, value_type=float),
                'box_width': ParameterValue(box_width, value_type=float),
                'box_height': ParameterValue(box_height, value_type=float),
                'marker_ns': 'opponent_vehicle_3',
                'marker_id': 3,
                'color_rgba': [0.6, 1.0, 0.2, 0.9],
            }],
            output='screen',
            condition=IfCondition(use_opponent_marker),
        ),
    ])
