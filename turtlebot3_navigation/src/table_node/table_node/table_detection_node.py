import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
import math

# params
MIN_TABLE_CLUSTER_SIZE = 4
MAX_TABLE_CLUSTER_SIZE = 14
DISTANCE_THRESHOLD = 1.0
DETECTION_RATE_HZ = 1   

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        self.table_count_pub = self.create_publisher(Int32, '/tables', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        # store latest scan
        self.latest_scan = None

        # run detection at fixed rate
        self.timer = self.create_timer(1.0 / DETECTION_RATE_HZ, self.timer_callback)

        self.get_logger().info("Table detection node started!")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is None:
            return
        self.process_scan(self.latest_scan)

    def polar_to_cartesian(self, r, theta):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y

    def process_scan(self, msg: LaserScan):
        cluster = []
        detected_tables = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment
            x, y = self.polar_to_cartesian(r, theta)

            if cluster:
                prev_x, prev_y = cluster[-1]
                distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                if distance > DISTANCE_THRESHOLD:
                    if MIN_TABLE_CLUSTER_SIZE <= len(cluster) <= MAX_TABLE_CLUSTER_SIZE:
                        cx = sum(p[0] for p in cluster) / len(cluster)
                        cy = sum(p[1] for p in cluster) / len(cluster)
                        detected_tables.append((cx, cy))
                    cluster = []

            cluster.append((x, y))

        # last cluster
        if MIN_TABLE_CLUSTER_SIZE <= len(cluster) <= MAX_TABLE_CLUSTER_SIZE:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            detected_tables.append((cx, cy))

        # publish number of detected tables
        count_msg = Int32()
        count_msg.data = len(detected_tables)
        self.table_count_pub.publish(count_msg)

        self.get_logger().info(f"Detected {len(detected_tables)} tables.")

def main(args=None):
    rclpy.init(args=args)
    node = TableDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

