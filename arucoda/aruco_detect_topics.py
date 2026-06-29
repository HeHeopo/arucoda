#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from std_msgs.msg import Int32MultiArray, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R


class ArucoDetectTopicsNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_topics_node')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')

        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        dict_name = self.get_parameter('aruco_dict').value
        dict_attr = getattr(cv2.aruco, dict_name, None)
        if dict_attr is None:
            self.get_logger().error(f'Unknown ArUco dictionary: {dict_name}')
            raise RuntimeError(f'Unknown dictionary: {dict_name}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        self.R_cv_to_ros = np.array([
            [0,  0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ], dtype=np.float32)

        self.marker_positions = {}
        self.known_ids = set()
        self.smoothing_alpha = 0.3
        self.position_min_delta = 0.005

        self.camera_matrix = None
        self.dist_coeffs = None

        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.create_timer(0.2, self._process_and_publish_tfs)

        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, 'aruco_ids', 10)

        self.get_logger().info('ArUco detect topics node started')

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        frame_id = msg.header.frame_id
        if self.camera_frame:
            frame_id = self.camera_frame

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = frame_id

        id_array = Int32MultiArray()
        id_array.data = []

        for i, marker_id in enumerate(ids.flatten()):
            int_id = int(marker_id)
            self.known_ids.add(int_id)

            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            self._publish_raw_camera_tf(rvec, tvec, int_id, frame_id)

            R_cv, _ = cv2.Rodrigues(rvec)
            R_ros = self.R_cv_to_ros @ R_cv
            t_ros = self.R_cv_to_ros @ tvec
            quat = R.from_matrix(R_ros).as_quat()

            pose = Pose()
            pose.position.x = float(t_ros[0])
            pose.position.y = float(t_ros[1])
            pose.position.z = float(t_ros[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            pose_array.poses.append(pose)
            id_array.data.append(int_id)

        self.poses_pub.publish(pose_array)
        self.ids_pub.publish(id_array)

    def _publish_raw_camera_tf(self, rvec, tvec, marker_id: int, frame_id: str):
        R_cv, _ = cv2.Rodrigues(rvec)
        R_ros = self.R_cv_to_ros @ R_cv
        t_ros = self.R_cv_to_ros @ tvec
        quat = R.from_matrix(R_ros).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = f'cam_aruco_{marker_id}'
        t.transform.translation.x = float(t_ros[0])
        t.transform.translation.y = float(t_ros[1])
        t.transform.translation.z = float(t_ros[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _process_and_publish_tfs(self):
        for marker_id in list(self.known_ids):
            try:
                map_tf = self.tf_buffer.lookup_transform(
                    self.map_frame, f'cam_aruco_{marker_id}',
                    rclpy.time.Time()
                )
                raw_x = map_tf.transform.translation.x
                raw_y = map_tf.transform.translation.y

                if marker_id not in self.marker_positions:
                    self.marker_positions[marker_id] = {'x': raw_x, 'y': raw_y}
                else:
                    prev = self.marker_positions[marker_id]
                    dx = abs(raw_x - prev['x'])
                    dy = abs(raw_y - prev['y'])
                    if dx > self.position_min_delta or dy > self.position_min_delta:
                        a = self.smoothing_alpha
                        prev['x'] = a * raw_x + (1 - a) * prev['x']
                        prev['y'] = a * raw_y + (1 - a) * prev['y']
            except Exception:
                pass

        for mid, pos in self.marker_positions.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = f'aruco_marker_{mid}'
            t.transform.translation.x = float(pos['x'])
            t.transform.translation.y = float(pos['y'])
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectTopicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
