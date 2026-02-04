from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


NUM_NPCS = 9

# NPC colors (distinct colors for each NPC)
NPC_COLORS = [
    [0.2, 0.6, 1.0, 0.9],   # NPC_1: Blue
    [1.0, 0.6, 0.2, 0.9],   # NPC_2: Orange
    [0.6, 1.0, 0.2, 0.9],   # NPC_3: Lime
    [1.0, 0.2, 0.6, 0.9],   # NPC_4: Pink
    [0.2, 1.0, 0.8, 0.9],   # NPC_5: Cyan
    [0.8, 0.2, 1.0, 0.9],   # NPC_6: Purple
    [1.0, 1.0, 0.2, 0.9],   # NPC_7: Yellow
    [0.4, 0.4, 0.8, 0.9],   # NPC_8: Slate Blue
    [0.8, 0.4, 0.4, 0.9],   # NPC_9: Salmon
]


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    frame_id = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_vehicle_marker = LaunchConfiguration('use_vehicle_marker')
    use_npc_markers = LaunchConfiguration('use_npc_markers')
    use_waypoints_path = LaunchConfiguration('use_waypoints_path')
    use_static_map_odom_tf = LaunchConfiguration('use_static_map_odom_tf')
    use_static_world_map_tf = LaunchConfiguration('use_static_world_map_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    waypoints_csv = LaunchConfiguration('waypoints_csv')
    box_length = LaunchConfiguration('box_length')
    box_width = LaunchConfiguration('box_width')
    box_height = LaunchConfiguration('box_height')
    use_msg_frame = LaunchConfiguration('use_msg_frame')

    # NPC odom topic configurations
    npc_odom_topics = [LaunchConfiguration(f'npc_{i}_odom_topic') for i in range(1, NUM_NPCS + 1)]

    launch_items = [
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
            description='Publish ego vehicle box marker',
        ),
        DeclareLaunchArgument(
            'use_npc_markers',
            default_value='true',
            description='Publish NPC vehicle box markers',
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
            default_value='/ego/odom',
            description='Odometry topic for ego vehicle pose',
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
    ]

    # NPC odom topic launch arguments
    for i in range(1, NUM_NPCS + 1):
        launch_items.append(
            DeclareLaunchArgument(
                f'npc_{i}_odom_topic',
                default_value=f'/NPC_{i}/odom',
                description=f'Odometry topic for NPC_{i} pose',
            )
        )

    # Core nodes
    launch_items.extend([
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
        # Ego vehicle marker
        Node(
            package='morai_visualizer',
            executable='vehicle_marker',
            name='ego_vehicle_marker',
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
    ])

    # NPC vehicle markers
    for i in range(NUM_NPCS):
        npc_id = i + 1
        launch_items.append(
            Node(
                package='morai_visualizer',
                executable='vehicle_marker',
                name=f'npc_{npc_id}_marker',
                parameters=[{
                    'odom_topic': npc_odom_topics[i],
                    'frame_id': frame_id,
                    'use_msg_frame': ParameterValue(use_msg_frame, value_type=bool),
                    'box_length': ParameterValue(box_length, value_type=float),
                    'box_width': ParameterValue(box_width, value_type=float),
                    'box_height': ParameterValue(box_height, value_type=float),
                    'marker_ns': f'npc_{npc_id}_vehicle',
                    'marker_id': npc_id,
                    'color_rgba': NPC_COLORS[i],
                }],
                output='screen',
                condition=IfCondition(use_npc_markers),
            )
        )

    return LaunchDescription(launch_items)
