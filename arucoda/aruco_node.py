#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('fps', 24)
        self.declare_parameter('use_v4l2', True)

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.03)
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('fx', 836.51438028)
        self.declare_parameter('fy', 835.73816422)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('k1', -0.25977023)
        self.declare_parameter('k2', 0.17973683)
        self.declare_parameter('p1', -0.00320421)
        self.declare_parameter('p2', -0.00957875)
        self.declare_parameter('k3', -0.086389)

        self.declare_parameter('width', 0.0)
        self.declare_parameter('height', 0.0)

        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('position_min_delta', 0.005)

        camera_id = self.get_parameter('camera_id').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        use_v4l2 = self.get_parameter('use_v4l2').value

        dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx_param = self.get_parameter('cx').value
        cy_param = self.get_parameter('cy').value
        k1 = self.get_parameter('k1').value
        k2 = self.get_parameter('k2').value
        p1 = self.get_parameter('p1').value
        p2 = self.get_parameter('p2').value
        k3 = self.get_parameter('k3').value

        w_param = self.get_parameter('width').value
        h_param = self.get_parameter('height').value

        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.position_min_delta = self.get_parameter('position_min_delta').value

        dict_attr = getattr(cv2.aruco, dict_name, None)
        if dict_attr is None:
            self.get_logger().error(f'Unknown ArUco dictionary: {dict_name}')
            raise RuntimeError(f'Unknown dictionary: {dict_name}')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_attr)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()

        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2 if use_v4l2 else cv2.CAP_ANY)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        if w_param == 0.0:
            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        else:
            self.width = int(w_param)
        if h_param == 0.0:
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        else:
            self.height = int(h_param)

        if cx_param == 0.0:
            cx = self.width / 2.0
        else:
            cx = cx_param
        if cy_param == 0.0:
            cy = self.height / 2.0
        else:
            cy = cy_param

        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.R_cv_to_ros = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ], dtype=np.float32)

        self.marker_positions = {}
        self.known_ids = set()

        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, 'aruco_ids', 10)

        self.create_timer(0.2, self._process_and_publish_tfs)
        self.create_timer(1.0 / self.fps, self.process_frame)

        self.get_logger().info(
            f'ArucoNode started — camera {camera_id} ({self.width}x{self.height})'
        )

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.camera_frame
        id_array = Int32MultiArray()
        id_array.data = []

        for i, marker_id in enumerate(ids.flatten()):
            int_id = int(marker_id)
            self.known_ids.add(int_id)
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            R_cv, _ = cv2.Rodrigues(rvec)
            R_ros = self.R_cv_to_ros @ R_cv
            t_ros = self.R_cv_to_ros @ tvec
            quat = R.from_matrix(R_ros).as_quat()

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f'cam_aruco_{int_id}'
            t.transform.translation.x = float(t_ros[0])
            t.transform.translation.y = float(t_ros[1])
            t.transform.translation.z = float(t_ros[2])
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(t)

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

            dist = np.linalg.norm(tvec)
            self.get_logger().info(f'ID {int_id}: {dist:.2f} m', throttle_duration_sec=1.0)

        self.poses_pub.publish(pose_array)
        self.ids_pub.publish(id_array)

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

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
