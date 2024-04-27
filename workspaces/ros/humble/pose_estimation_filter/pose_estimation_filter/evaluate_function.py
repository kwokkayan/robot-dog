import cv2
import rclpy
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose


class PoseEstimationFilter(Node):
    def __init__(self):
        super().__init__("pose_estimation_filter")

        self.bridge = CvBridge()
        
        self.camera_info = None

        self.mask_image = None
        self.depth_image = None

        self.qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            "/camera/depth/camera_info",
            self.camera_info_listener_callback,
            self.qos_policy,
        )

        self.mask_subscription = self.create_subscription(
            Image,
            "/unet/raw_segmentation_mask_depadded",
            self.mask_listener_callback,
            self.qos_policy,
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/depth/image_rect_raw",
            self.depth_listener_callback,
            self.qos_policy,
        )

        self.publisher = self.create_publisher(
            PointStamped,
            "/clicked_point",
            10
        )

        self.timer = self.create_timer(1, self.evaluate)

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        ) 
    
    def camera_info_listener_callback(self, msg):
        try:
            self.camera_info = msg
        except:
            self.get_logger().error("Camera info listener callack exception")

    def mask_listener_callback(self, msg):
        try:
            self.mask_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("Mask listener callack exception: {}".format(e))

    def depth_listener_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error("Depth listener callack exception: {}".format(e))

    def evaluate(self):
        if self.camera_info is None or self.mask_image is None or self.depth_image is None:
            return
    
        ret, mask = cv2.threshold(self.mask_image, 0, 1, cv2.THRESH_BINARY)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=4)
        
        if num_labels <= 1:
            return

        largest_label = 1 + np.argmax(stats[1: , cv2.CC_STAT_AREA])

        x, y = centroids[largest_label]

        d = self.depth_image[int(y), int(x)]
        
        if d == 0:
            return 

        intrinsics = rs.intrinsics()

        intrinsics.width = self.camera_info.width 
        intrinsics.height = self.camera_info.height

        intrinsics.ppx = self.camera_info.k[2]
        intrinsics.ppy = self.camera_info.k[5]
        intrinsics.fx = self.camera_info.k[0]
        intrinsics.fy = self.camera_info.k[4]
   
        intrinsics.model = rs.distortion.none

        intrinsics.coeffs = [i for i in self.camera_info.d]

        coords = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)

        point_stamped = PointStamped()

        point_stamped.header.stamp = self.get_clock().now().to_msg()

        point_stamped.header.frame_id = "camera_depth_optical_frame"

        point_stamped.point.x = coords[0] / 1000
        point_stamped.point.y = coords[1] / 1000
        point_stamped.point.z = coords[2] / 1000

        self.publisher.publish(point_stamped)

        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml"

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "odom"

        goal_msg.pose.pose.position.x = coords[0] / 1000
        goal_msg.pose.pose.position.y = coords[1] / 1000
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    pose_estimation_filter = PoseEstimationFilter()

    rclpy.spin(pose_estimation_filter)


if __name__ == '__main__':
    main()
