import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
import cv2

class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.bridge = CvBridge()
        self.latest_pc = None

        self.model = YOLO('yolov8n.pt')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_img = self.create_subscription(
            Image,
            '/fr3/depth_camera/image',
            self.image_cb,
            10)

        self.sub_pc = self.create_subscription(
            PointCloud2,
            '/fr3/depth_camera/points',
            self.pc_cb,
            10)

        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/cube_pose',
            10)

        self.get_logger().info('Cube detector started')

    def pc_cb(self, msg):
        self.latest_pc = msg

    def find_red_cube(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # range rosso ampio
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([155, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # prendi il contorno più grande
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        self.get_logger().info(f'Red contour area: {area:.0f}')

        if area < 100:
            return None

        x, y, w, h = cv2.boundingRect(c)
        return (x, y, x+w, y+h)

    def image_cb(self, msg):
        if self.latest_pc is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        bbox = self.find_red_cube(frame)

        if bbox is None:
            self.get_logger().warn('No red cube found', throttle_duration_sec=2.0)
            return

        u1, v1, u2, v2 = bbox
        u = int((u1 + u2) / 2)
        v = int((v1 + v2) / 2)

        self.get_logger().info(f'Red cube centroid at pixel ({u}, {v})')

        width = self.latest_pc.width
        point_idx = v * width + u

        points = list(pc2.read_points(
            self.latest_pc,
            field_names=('x', 'y', 'z'),
            skip_nans=False))

        if point_idx >= len(points):
            self.get_logger().warn('Point index out of range')
            return

        x, y, z = points[point_idx]

        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            self.get_logger().warn('NaN point at centroid')
            return

        pose_camera = PoseStamped()
        pose_camera.header.stamp = self.latest_pc.header.stamp
        pose_camera.header.frame_id = 'fr3/rgbd_camera_frame'
        pose_camera.pose.position.x = float(x)
        pose_camera.pose.position.y = float(y)
        pose_camera.pose.position.z = float(z)
        pose_camera.pose.orientation.w = 1.0

        try:
            pose_world = self.tf_buffer.transform(
                pose_camera, 'world',
                timeout=rclpy.duration.Duration(seconds=1.0))

            self.pub_pose.publish(pose_world)
            self.get_logger().info(
                f'Cube pose in world: '
                f'x={pose_world.pose.position.x:.3f} '
                f'y={pose_world.pose.position.y:.3f} '
                f'z={pose_world.pose.position.z:.3f}')

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
