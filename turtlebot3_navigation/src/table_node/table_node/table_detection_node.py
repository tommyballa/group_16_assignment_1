import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math

# Parameters
MIN_TABLE_CLUSTER_SIZE = 4      # LIDAR pts for a table
MAX_TABLE_CLUSTER_SIZE = 14     # Bigger ones are not tables
RANGE_JUMP_THRESHOLD = 0.15      # Break cluster if > 15 cm
DETECTION_RATE_HZ = 1


class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        # Publish number of detected tables
        self.table_count_pub = self.create_publisher(Int32, '/tables', 10)

        # Sub to LIDAR, call callback at every scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None     # Latest scan
        # Avoid using LIDAR frequency
        self.timer = self.create_timer(1.0 / DETECTION_RATE_HZ, self.timer_callback)
        self.get_logger().info("Table detection node started (range clustering).")

    # Save latest LIDAR scan
    def scan_callback(self, msg):
        self.latest_scan = msg

    # Check for published scan
    def timer_callback(self):
        if self.latest_scan is not None:
            self.process_scan(self.latest_scan)

    def compute_centroid(self, ranges, angles):
        xs = []
        ys = []
        for r, th in zip(ranges, angles):       # From (range, angle) to coords
            xs.append(r * math.cos(th))
            ys.append(r * math.sin(th))
        # Get middle point
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def process_scan(self, msg: LaserScan):
        # Accumulate for current cluster
        cluster_ranges = []
        cluster_angles = []
        detected_tables = []

        prev_r = None       # Check for discontinuity

        # Skip invalid values
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                prev_r = None
                continue

            # Angle of current reading
            theta = msg.angle_min + i * msg.angle_increment

            # Start a new cluster
            if prev_r is None:
                cluster_ranges = [r]
                cluster_angles = [theta]
                prev_r = r
                continue
            
            # Break the cluster if too far from the previous scan
            if abs(r - prev_r) > RANGE_JUMP_THRESHOLD:
                if MIN_TABLE_CLUSTER_SIZE <= len(cluster_ranges) <= MAX_TABLE_CLUSTER_SIZE:
                    detected_tables.append(
                        self.compute_centroid(cluster_ranges, cluster_angles)
                    )
                # Start a new cluster
                cluster_ranges = [r]
                cluster_angles = [theta]
            # Else append to the existing cluster
            else:
                cluster_ranges.append(r)
                cluster_angles.append(theta)

            prev_r = r      # Update last distance

        # Check cluster size
        if MIN_TABLE_CLUSTER_SIZE <= len(cluster_ranges) <= MAX_TABLE_CLUSTER_SIZE:
            detected_tables.append(
                self.compute_centroid(cluster_ranges, cluster_angles)
            )
        # Publish result
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
