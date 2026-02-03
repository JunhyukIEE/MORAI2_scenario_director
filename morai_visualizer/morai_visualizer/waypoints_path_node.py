import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class WaypointsPathNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoints_path_node')

        self.declare_parameter('waypoints_csv', 'waypoints.csv')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('topic', '/waypoints_path')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._publisher = self.create_publisher(NavPath, topic, qos)

        csv_path = self.get_parameter('waypoints_csv').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self._path_msg = self._load_path(csv_path)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        period = 1.0 / max(rate_hz, 0.1)
        self.create_timer(period, self._publish)

    def _load_path(self, csv_path: str) -> NavPath:
        path = NavPath()
        path.header.frame_id = self._frame_id

        path_file = Path(csv_path)
        if not path_file.exists():
            self.get_logger().error(f"Waypoints CSV not found: {csv_path}")
            return path

        with path_file.open('r', newline='') as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                self.get_logger().error(f"Waypoints CSV has no header: {csv_path}")
                return path
            for row in reader:
                try:
                    x = float(row.get('x', 'nan'))
                    y = float(row.get('y', 'nan'))
                    yaw = float(row.get('yaw', '0.0'))
                except ValueError:
                    continue
                if math.isnan(x) or math.isnan(y):
                    continue

                pose = PoseStamped()
                pose.header.frame_id = self._frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation = yaw_to_quaternion(yaw)
                path.poses.append(pose)

        if not path.poses:
            self.get_logger().warn(f"No waypoints loaded from: {csv_path}")
        else:
            self.get_logger().info(f"Loaded {len(path.poses)} waypoints from {csv_path}")
        return path

    def _publish(self) -> None:
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self._path_msg.poses:
            pose.header.stamp = self._path_msg.header.stamp
        self._publisher.publish(self._path_msg)


def main() -> None:
    rclpy.init()
    node = WaypointsPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
