#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

class ComputeGoalNode(Node):
    def __init__(self):
        super().__init__('compute_goal_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detections_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.declare_parameter('target_frame', 'map')  
        self.declare_parameter('source_frame', 'external_camera/link/rgb_camera')  

    def detections_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            self.get_logger().info('No tags detected.')
            return

        sum_x, sum_y = 0.0, 0.0
        for detection in msg.detections:
            sum_x += detection.centre.x
            sum_y += detection.centre.y

        midpoint_cam = PointStamped()
        midpoint_cam.header = msg.header
        midpoint_cam.point.x = sum_x / len(msg.detections)
        midpoint_cam.point.y = sum_y / len(msg.detections)
        midpoint_cam.point.z = 0.0

        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        source_frame = self.get_parameter('source_frame').get_parameter_value().string_value

        try:
            # wait till transform ready
            if self.tf_buffer.can_transform(target_frame,
                                            source_frame,
                                            rclpy.time.Time(),
                                            timeout=rclpy.duration.Duration(seconds=1.0)):
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
                midpoint_map = do_transform_point(midpoint_cam, transform)

                # publish Nav2 PoseStamped
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = target_frame
                pose.pose.position = midpoint_map.point
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                self.publisher.publish(pose)
                self.get_logger().info(
                    f'Published Nav2 goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
                )
            else:
                self.get_logger().warn(
                    f'TF transform from {source_frame} to {target_frame} not yet available.'
                )

        except TransformException as e:
            self.get_logger().warn(f'TF transform failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ComputeGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

