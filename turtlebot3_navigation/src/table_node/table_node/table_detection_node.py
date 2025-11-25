import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math

# params
MIN_TABLE_CLUSTER_SIZE = 4
MAX_TABLE_CLUSTER_SIZE = 14
RANGE_JUMP_THRESHOLD = 0.15      # break cluster if > 15 cm
DETECTION_RATE_HZ = 1


class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        self.table_count_pub = self.create_publisher(Int32, '/tables', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None
        self.timer = self.create_timer(1.0 / DETECTION_RATE_HZ, self.timer_callback)
        self.get_logger().info("Table detection node started (range clustering).")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is not None:
            self.process_scan(self.latest_scan)

    def compute_centroid(self, ranges, angles):
        xs = []
        ys = []
        for r, th in zip(ranges, angles):
            xs.append(r * math.cos(th))
            ys.append(r * math.sin(th))
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def process_scan(self, msg: LaserScan):
        cluster_ranges = []
        cluster_angles = []
        detected_tables = []

        prev_r = None

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                prev_r = None
                continue

            theta = msg.angle_min + i * msg.angle_increment

            if prev_r is None:
                cluster_ranges = [r]
                cluster_angles = [theta]
                prev_r = r
                continue

            if abs(r - prev_r) > RANGE_JUMP_THRESHOLD:
                if MIN_TABLE_CLUSTER_SIZE <= len(cluster_ranges) <= MAX_TABLE_CLUSTER_SIZE:
                    detected_tables.append(
                        self.compute_centroid(cluster_ranges, cluster_angles)
                    )

                cluster_ranges = [r]
                cluster_angles = [theta]

            else:
                cluster_ranges.append(r)
                cluster_angles.append(theta)

            prev_r = r

        if MIN_TABLE_CLUSTER_SIZE <= len(cluster_ranges) <= MAX_TABLE_CLUSTER_SIZE:
            detected_tables.append(
                self.compute_centroid(cluster_ranges, cluster_angles)
            )
        msg_out = Int32()
        msg_out.data = len(detected_tables)
        self.table_count_pub.publish(msg_out)

        self.get_logger().info(f"[Detection] Tables: {len(detected_tables)}")


def main(args=None):
    rclpy.init(args=args)
    node = TableDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
