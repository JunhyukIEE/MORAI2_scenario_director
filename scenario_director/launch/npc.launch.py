from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    # NPC 파라미터는 npc_controller.yaml에 모두 포함 (pure_pursuit 포함)
    base_params = [
        f"{share}/config/npc_controller.yaml",
    ]

    nodes = []

    for npc_id in range(1, 10):
        node = Node(
            package='scenario_director',
            executable='npc_controller_node',
            name=f'npc_{npc_id}_controller',
            parameters=base_params + [
                {'npc.id': npc_id},
            ],
            output='screen'
        )
        nodes.append(node)

    return LaunchDescription(nodes)
