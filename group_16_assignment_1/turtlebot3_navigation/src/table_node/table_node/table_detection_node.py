import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
import math

# --- OPTIMIZED CONFIGURATION PARAMETERS ---
MIN_TABLE_CLUSTER_SIZE = 2      # Lowered to 2 to see dots up above
MAX_TABLE_CLUSTER_SIZE = 20     # Slightly higher
RANGE_JUMP_THRESHOLD = 0.15     # Skip in mt to separate objects (e.g. cylinder from wall)
DETECTION_RATE_HZ = 1           
MAX_DETECTION_DISTANCE = 3.5   # Avoid everything higher than 5 mt
MAX_CYLINDER_WIDTH = 0.6        # If an object is larger than 60cm, it's a wall not a cylynder

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        # Counter publisher
        self.table_count_pub = self.create_publisher(Int32, '/tables', 10)
        
        # Visualization publisher (Rviz)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Laser subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None
        self.timer = self.create_timer(1.0 / DETECTION_RATE_HZ, self.timer_callback)
        
        self.get_logger().info(f"Nodo avviato. Min Punti: {MIN_TABLE_CLUSTER_SIZE}, Filtro Larghezza: {MAX_CYLINDER_WIDTH}m")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is not None:
            self.process_scan(self.latest_scan)

    def compute_centroid(self, ranges, angles):
        """Calcola il centro geometrico del cluster."""
        xs = []
        ys = []
        for r, th in zip(ranges, angles):
            xs.append(r * math.cos(th))
            ys.append(r * math.sin(th))
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    def validate_cluster(self, ranges, angles):
        """
        Analizza un gruppo di punti e decide se è un cilindro valido o spazzatura (muro).
        """
        # Filter NUMBER OF PTS
        # If less than 2 pts or too many, discard
        if not (MIN_TABLE_CLUSTER_SIZE <= len(ranges) <= MAX_TABLE_CLUSTER_SIZE):
            return None

        # Compute coords for cluster start and end
        x_start = ranges[0] * math.cos(angles[0])
        y_start = ranges[0] * math.sin(angles[0])
        x_end = ranges[-1] * math.cos(angles[-1])
        y_end = ranges[-1] * math.sin(angles[-1])

        # Filter PHYSICAL DIMENSION (Euclidean)
        # If pts wider than 60cm, it's probably a diagonal wall
        width = math.sqrt((x_start - x_end)**2 + (y_start - y_end)**2)
        if width > MAX_CYLINDER_WIDTH:
            return None

        # Filter DISTANCE FROM ROBOT
        centroid = self.compute_centroid(ranges, angles)
        dist_from_robot = math.sqrt(centroid[0]**2 + centroid[1]**2)
        
        if dist_from_robot > MAX_DETECTION_DISTANCE:
            return None
        
        # If all tests passed, it's a cylinder
        return centroid

    def publish_markers(self, detected_tables, frame_id):
        """Pubblica i cilindri rossi su Rviz."""
        marker_array = MarkerArray()

        # Special marker to delete old drawings
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, (x, y) in enumerate(detected_tables):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cylinders"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.4 
            marker.scale.y = 0.4
            marker.scale.z = 0.5 

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8 

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def process_scan(self, msg: LaserScan):
        cluster_ranges = []
        cluster_angles = []
        detected_tables = []
        prev_r = None

        for i, r in enumerate(msg.ranges):
            # Discard invalid values
            if math.isinf(r) or math.isnan(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment

            if prev_r is None:
                cluster_ranges = [r]
                cluster_angles = [theta]
                prev_r = r
                continue
            
            # If distance jumps suddenly, cluster has ended
            if abs(r - prev_r) > RANGE_JUMP_THRESHOLD:
                # Check if the cluster we just closed is valid
                valid_centroid = self.validate_cluster(cluster_ranges, cluster_angles)
                if valid_centroid:
                    detected_tables.append(valid_centroid)

                # Reset for new cluster
                cluster_ranges = [r]
                cluster_angles = [theta]
            else:
                cluster_ranges.append(r)
                cluster_angles.append(theta)

            prev_r = r

        # Check last open cluster
        valid_centroid = self.validate_cluster(cluster_ranges, cluster_angles)
        if valid_centroid:
            detected_tables.append(valid_centroid)

        # Publish results
        msg_out = Int32()
        msg_out.data = len(detected_tables)
        self.table_count_pub.publish(msg_out)
        
        self.publish_markers(detected_tables, msg.header.frame_id)
        
        self.get_logger().info(f"Cilindri rilevati: {len(detected_tables)}")

def main(args=None):
    rclpy.init(args=args)
    node = TableDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
