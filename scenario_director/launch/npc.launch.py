from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('scenario_director')
    base_params = [
        f"{share}/config/npc_controller.yaml",
        f"{share}/config/pure_pursuit.yaml"
    ]

    # NPC speed ratios (can be customized per NPC)
    # NPC_1 is slowest, NPC_9 is fastest
    npc_speed_ratios = {
        1: 0.40,
        2: 0.45,
        3: 0.50,
        4: 0.55,
        5: 0.60,
        6: 0.65,
        7: 0.70,
        8: 0.75,
        9: 0.80,
    }

    nodes = []

    for npc_id in range(1, 10):
        speed_ratio = npc_speed_ratios.get(npc_id, 0.5)
        node = Node(
            package='scenario_director',
            executable='npc_controller_node',
            name=f'npc_{npc_id}_controller',
            parameters=base_params + [
                {'npc.id': npc_id},
                {'npc.speed_ratio': speed_ratio},
            ],
            output='screen'
        )
        nodes.append(node)

    return LaunchDescription(nodes)
