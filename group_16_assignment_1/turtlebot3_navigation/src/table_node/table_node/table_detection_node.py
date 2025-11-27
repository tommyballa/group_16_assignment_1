import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Bool 
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
import math

# OPTIMIZED CONFIGURATION PARAMETERS 
MIN_TABLE_CLUSTER_SIZE = 2      
MAX_TABLE_CLUSTER_SIZE = 20     
RANGE_JUMP_THRESHOLD = 0.15     
DETECTION_RATE_HZ = 1           
MAX_DETECTION_DISTANCE = 3.5   
MAX_CYLINDER_WIDTH = 0.6        

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')

        # Flag to start tables detectin
        self.navigation_completed = False

        # Counter publisher
        self.table_count_pub = self.create_publisher(Int32, '/tables', 10)
        
        # Coordinate publisher
        self.pose_pub = self.create_publisher(PoseArray, '/table_poses', 10)
        
        # Visualisation publisher 
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # TF handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber to signal goal reached
        self.status_sub = self.create_subscription(
            Bool, 
            'navigation_completed',
            self.status_callback,
            10
        )

        # Subscriber Laser
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None
        self.timer = self.create_timer(1.0 / DETECTION_RATE_HZ, self.timer_callback)
        
        self.get_logger().info(f"Node started. Waiting the robot to reach the goal pose.")


    def status_callback(self, msg):
        if msg.data is True and not self.navigation_completed:
            self.navigation_completed = True
            self.get_logger().info("Signal of succeed navigation received. Starting tables detection.")


    def scan_callback(self, msg):
        self.latest_scan = msg

    # Only if navigation is completed process the scan
    def timer_callback(self):

        if not self.navigation_completed:
            return

        if self.latest_scan is not None:
            self.process_scan(self.latest_scan)
    
    # Compute geometric centroid of the clusters
    def compute_centroid(self, ranges, angles):

        xs = []
        ys = []
        for r, th in zip(ranges, angles):
            xs.append(r * math.cos(th))
            ys.append(r * math.sin(th))
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    # Deciding if the cluster is valid or not
    def validate_cluster(self, ranges, angles):

        if not (MIN_TABLE_CLUSTER_SIZE <= len(ranges) <= MAX_TABLE_CLUSTER_SIZE):
            return None

        x_start = ranges[0] * math.cos(angles[0])
        y_start = ranges[0] * math.sin(angles[0])
        x_end = ranges[-1] * math.cos(angles[-1])
        y_end = ranges[-1] * math.sin(angles[-1])

        width = math.sqrt((x_start - x_end)**2 + (y_start - y_end)**2)
        if width > MAX_CYLINDER_WIDTH:
            return None

        centroid = self.compute_centroid(ranges, angles)
        dist_from_robot = math.sqrt(centroid[0]**2 + centroid[1]**2)
        
        if dist_from_robot > MAX_DETECTION_DISTANCE:
            return None
        
        return centroid

    # Print a red circle on the detected centroids.
    def publish_markers(self, detected_tables, frame_id):
        marker_array = MarkerArray()
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

    #Transform local coordinate in Odom and publish it on /table_poses
    def publish_odom_coordinates(self, detected_tables, scan_header):

        pose_array = PoseArray()
        pose_array.header.frame_id = "odom"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for (local_x, local_y) in detected_tables:
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.header = scan_header 
            pose_stamped.pose.position.x = local_x
            pose_stamped.pose.position.y = local_y
            pose_stamped.pose.orientation.w = 1.0 

            try:

                pose_transformed = self.tf_buffer.transform(pose_stamped, 'odom')
                pose_array.poses.append(pose_transformed.pose)

            except TransformException as ex:
                self.get_logger().warn(f'Error in transforming the coordinates')
                return

        self.pose_pub.publish(pose_array)

    def process_scan(self, msg: LaserScan):
        cluster_ranges = []
        cluster_angles = []
        detected_tables = []
        prev_r = None

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment

            if prev_r is None:
                cluster_ranges = [r]
                cluster_angles = [theta]
                prev_r = r
                continue
            
            if abs(r - prev_r) > RANGE_JUMP_THRESHOLD:
                valid_centroid = self.validate_cluster(cluster_ranges, cluster_angles)
                if valid_centroid:
                    detected_tables.append(valid_centroid)
                cluster_ranges = [r]
                cluster_angles = [theta]
            else:
                cluster_ranges.append(r)
                cluster_angles.append(theta)

            prev_r = r

        valid_centroid = self.validate_cluster(cluster_ranges, cluster_angles)
        if valid_centroid:
            detected_tables.append(valid_centroid)

        # Publish results only if navigation is completed
        msg_out = Int32()
        msg_out.data = len(detected_tables)
        self.table_count_pub.publish(msg_out)
        
        self.publish_markers(detected_tables, msg.header.frame_id)

        if detected_tables:
            self.publish_odom_coordinates(detected_tables, msg.header)
        
        self.get_logger().info(f"Number of tabeles detected: {len(detected_tables)}")

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
