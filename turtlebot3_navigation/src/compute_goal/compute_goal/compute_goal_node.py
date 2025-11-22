import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

class ComputeGoalNode(Node):
    def __init__(self):
        super().__init__('compute_goal_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseStamped, '/goal', 10)
        self.target_frame = 'map'
        self.tag_frames = ['tag36h11:1', 'tag36h11:10']

        self.goal_published = False  

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.goal_published:
            return  

        now = Time()
        positions = []

        all_tags_available = all(
            self.tf_buffer.can_transform(self.target_frame, tag_frame, now)
            for tag_frame in self.tag_frames
        )

        if not all_tags_available:
            self.get_logger().info("Waiting for both tag TFs to be available...")
            return  

        for tag_frame in self.tag_frames:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                tag_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            positions.append(transform.transform.translation)

        mid_x = (positions[0].x + positions[1].x) / 2.0
        mid_y = (positions[0].y + positions[1].y) / 2.0
        mid_z = (positions[0].z + positions[1].z) / 2.0

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.target_frame
        goal.pose.position = Point(x=mid_x, y=mid_y, z=mid_z)
        goal.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher.publish(goal)
        self.get_logger().info(f"Published midpoint goal once: x={mid_x:.2f}, y={mid_y:.2f}, z={mid_z:.2f}")

        self.goal_published = True  


def main(args=None):
    rclpy.init(args=args)
    node = ComputeGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
