import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class VehicleMarkerNode(Node):
    def __init__(self):
        super().__init__('vehicle_marker_node')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('use_msg_frame', True)
        self.declare_parameter('box_length', 1.8)
        self.declare_parameter('box_width', 1.5)
        self.declare_parameter('box_height', 0.8)
        self.declare_parameter('color_rgba', [0.9, 0.2, 0.2, 0.9])
        self.declare_parameter('marker_ns', 'vehicle')
        self.declare_parameter('marker_id', 0)

        self._publisher = self.create_publisher(Marker, 'vehicle_marker', 1)
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        marker = Marker()
        marker.ns = self.get_parameter('marker_ns').get_parameter_value().string_value
        marker.id = self.get_parameter('marker_id').get_parameter_value().integer_value
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        use_msg_frame = self.get_parameter('use_msg_frame').get_parameter_value().bool_value
        marker.header.frame_id = msg.header.frame_id if use_msg_frame else self.get_parameter('frame_id').value
        marker.header.stamp = msg.header.stamp

        marker.pose = msg.pose.pose

        marker.scale.x = float(self.get_parameter('box_length').value)
        marker.scale.y = float(self.get_parameter('box_width').value)
        marker.scale.z = float(self.get_parameter('box_height').value)

        color = self.get_parameter('color_rgba').value
        if isinstance(color, (list, tuple)) and len(color) == 4:
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = float(color[3])
        else:
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.9

        self._publisher.publish(marker)


def main():
    rclpy.init()
    node = VehicleMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
