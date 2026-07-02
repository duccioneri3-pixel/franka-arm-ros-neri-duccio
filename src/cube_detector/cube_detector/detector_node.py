import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registra la trasformazione di PoseStamped)
import cv2


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.bridge = CvBridge()
        self.latest_pc = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS sensor_data: compatibile con publisher best-effort (camera reali/sim)
        self.sub_img = self.create_subscription(
            Image,
            '/fr3/depth_camera/image',
            self.image_cb,
            qos_profile_sensor_data)

        self.sub_pc = self.create_subscription(
            PointCloud2,
            '/fr3/depth_camera/points',
            self.pc_cb,
            qos_profile_sensor_data)

        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/cube_pose',
            10)

        self.get_logger().info('Cube detector started')
        self.enabled = True
        self.sub_enable = self.create_subscription(
            Bool, '/detector_enable', self.enable_cb, 10)

    def enable_cb(self, msg):
        if self.enabled != msg.data:
            self.get_logger().info(
                'Detection ' + ('abilitata' if msg.data else 'disabilitata'))
        self.enabled = msg.data

    def pc_cb(self, msg):
        self.latest_pc = msg

    def find_red_cube(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        self.get_logger().debug(f'Red contour area: {area:.0f}')

        if area < 100:
            return None

        x, y, w, h = cv2.boundingRect(c)
        return (x, y, x + w, y + h)

    def sample_point(self, u, v, half=4):
        """Campiona una finestra attorno a (u,v) e ritorna la mediana
        dei punti 3D validi. Robusto a NaN e rumore depth."""
        pc = self.latest_pc
        h_img = pc.height
        w_img = pc.width

        if h_img <= 1:
            self.get_logger().warn(
                'Point cloud non organized: indicizzazione (u,v) non valida')
            return None

        # lettura completa, indicizzazione manuale (compatibile sensor_msgs_py Humble)
        all_pts = list(pc2.read_points(
            pc, field_names=('x', 'y', 'z'), skip_nans=False))

        pts = []
        for vv in range(max(0, v - half), min(h_img, v + half + 1)):
            for uu in range(max(0, u - half), min(w_img, u + half + 1)):
                idx = vv * w_img + uu
                if 0 <= idx < len(all_pts):
                    x, y, z = all_pts[idx]
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        pts.append([x, y, z])

        if not pts:
            return None

        return np.median(np.array(pts, dtype=np.float64), axis=0)

    def image_cb(self, msg):
        if not self.enabled:
            return
        if self.latest_pc is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}',
                                   throttle_duration_sec=2.0)
            return

        bbox = self.find_red_cube(frame)

        if bbox is None:
            self.get_logger().warn('No red cube found', throttle_duration_sec=2.0)
            return

        u1, v1, u2, v2 = bbox
        u = int((u1 + u2) / 2)
        v = int((v1 + v2) / 2)

        self.get_logger().debug(f'Red cube centroid at pixel ({u}, {v})')

        point = self.sample_point(u, v)
        if point is None:
            self.get_logger().warn('No valid 3D point near centroid',
                                   throttle_duration_sec=2.0)
            return

        x, y, z = float(point[0]), float(point[1]), float(point[2])

        pose_camera = PoseStamped()
        pose_camera.header.stamp = self.latest_pc.header.stamp
        pose_camera.header.frame_id = 'fr3/rgbd_camera_frame'
        pose_camera.pose.position.x = x
        pose_camera.pose.position.y = y
        pose_camera.pose.position.z = z
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
            self.get_logger().warn(f'TF transform failed: {e}',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
