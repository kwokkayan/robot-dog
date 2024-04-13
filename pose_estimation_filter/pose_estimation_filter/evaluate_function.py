import cv2
import rclpy
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped


class PoseEstimationFilter(Node):
    def __init__(self):
        super().__init__("pose_estimation_filter")

        self.bridge = CvBridge()
        
        self.camera_info = None

        self.mask_image = None
        self.depth_image = None

        self.camera_info_subscription = self.create_subscription(
            Image,
            "/unet/raw_segmentation_mask",
            self.camera_info_listener_callback,
            10
        )

        self.mask_subscription = self.create_subscription(
            Image,
            "/unet/raw_segmentation_mask",
            self.mask_listener_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointStamped,
            "/clicked_point",
            10
        )

        self.timer = self.create_timer(1, self.evaluate)
    
    def camera_info_listener_callback(self, msg):
        try:
            self.camera_info = msg
        except:
            self.get_logger().error("Camera info listener callack exception")

    def mask_listener_callback(self, msg):
        try:
            self.mask_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.get_logger().info("get mask")
        except CvBridgeError as e:
            self.get_logger().error("Mask listener callack exception: {}".format(e))

    def depth_listener_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.get_logger().info("get depth")
        except CvBridgeError as e:
            self.get_logger().error("Depth listener callack exception: {}".format(e))

    def evaluate(self):
        if self.camera_info is None or self.mask_image is None or self.depth_image is None:
            return
        
        self.get_logger().info("start evaluating")
        
        ret, mask = cv2.threshold(self.mask_image, 0, 1, cv2.THRESH_BINARY)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        self.get_logger().info(num_labels)
        if num_labels <= 1:
            return

        largest_label = 1 + np.argmax(stats[1: , cv2.CC_STAT_AREA])

        x, y = centroids[largest_label]

        d = self.depth_image[x][y]
        self.get_logger().info(x, y, d)

        intrinsics = rs.intrinsics()

        intrinsics.width = self.camera_info.width 
        intrinsics.height = self.camera_info.height

        intrinsics.ppx = self.camera_info.K[2]
        intrinsics.ppy = self.camera_info.K[5]
        intrinsics.fx = self.camera_info.K[0]
        intrinsics.fy = self.camera_info.K[4]
   
        intrinsics.model = rs.distortion.none

        intrinsics.coeffs = [i for i in self.camear_info.D]

        coords = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
        
        point_stamped = PointStamped()

        point_stamped.header.stamp = self.get_clock().now().to_msg()

        point_stamped.header.frame_id = "camera_depth_optical_frame"

        point_stamped.point.x = coords[0]
        point_stamped.point.y = coords[1]
        point_stamped.point.z = coords[2]

        self.publisher.pusblish(point_stamped)


def main(args=None):
    rclpy.init(args=args)

    pose_estimation_filter = PoseEstimationFilter()

    rclpy.spin(pose_estimation_filter)


if __name__ == '__main__':
    main()
