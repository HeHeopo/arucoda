import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import math


class ArucoCircleNode(Node):
    def __init__(self):
        super().__init__('aruco_circle')

        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('spin_angular_speed', 0.4)
        self.declare_parameter('total_markers', 4)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('parent_frame', 'camera_link')

        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('initial_pose_delay', 3.0)
        self.declare_parameter('approach_duration', 3.0)
        self.declare_parameter('approach_velocity', 0.3)

        dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.spin_angular = self.get_parameter('spin_angular_speed').value
        self.total_markers = self.get_parameter('total_markers').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.parent_frame = self.get_parameter('parent_frame').value

        ip_x = self.get_parameter('initial_pose_x').value
        ip_y = self.get_parameter('initial_pose_y').value
        ip_yaw = self.get_parameter('initial_pose_yaw').value
        ip_delay = self.get_parameter('initial_pose_delay').value
        self.approach_duration = self.get_parameter('approach_duration').value
        self.approach_velocity = self.get_parameter('approach_velocity').value

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

        self.camera_matrix = None
        self.dist_coeffs = None

        self.visited_markers = set()
        self.marker_positions = {}
        self.state = 'SPINNING'
        self._target_marker_id = None
        self._approach_dir = None
        self._approach_start = None
        self.wait_timer = None

        self._goal_handle = None
        self._nav_goal_sent = False

        self.R_cv_to_ros = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ], dtype=np.float32)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, 10
        )
        self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, 10
        )

        self.control_timer = self.create_timer(0.1, self.control_loop)
        self._initial_pose_timer = self.create_timer(ip_delay, self._publish_initial_pose_once)
        self._ip_x, self._ip_y, self._ip_yaw = ip_x, ip_y, ip_yaw

        self.get_logger().info(
            f'ArucoCircle started — spinning at {self.spin_angular} rad/s, '
            f'searching for {self.total_markers} ArUco markers via Nav2'
        )

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Initial pose published: x={x}, y={y}, yaw={yaw}')

    def _publish_initial_pose_once(self):
        self._initial_pose_timer.cancel()
        self.publish_initial_pose(self._ip_x, self._ip_y, self._ip_yaw)
        del self._initial_pose_timer

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        if self.state not in ('SPINNING', 'NAVIGATING', 'APPROACHING'):
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        frame_id = msg.header.frame_id
        for i, marker_id in enumerate(ids.flatten()):
            self._publish_marker_tf(rvecs[i][0], tvecs[i][0], int(marker_id), frame_id)

        if self.state == 'SPINNING':
            for i, marker_id in enumerate(ids.flatten()):
                int_id = int(marker_id)
                if int_id in self.visited_markers:
                    continue
                if int_id == self._target_marker_id:
                    continue
                if tvecs[i][0][2] <= 0:
                    continue
                self._target_marker_id = int_id
                self._approach_start = self.get_clock().now()
                self.state = 'APPROACHING'
                return

    def _quat_to_matrix(self, qx, qy, qz, qw):
        x2 = qx + qx
        y2 = qy + qy
        z2 = qz + qz
        xx = qx * x2
        xy = qx * y2
        xz = qx * z2
        yy = qy * y2
        yz = qy * z2
        zz = qz * z2
        wx = qw * x2
        wy = qw * y2
        wz = qw * z2
        return np.array([
            [1 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1 - (xx + yy)],
        ], dtype=np.float64)

    def _matrix_to_quat(self, R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2.0
            return (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4.0
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            return s / 4.0, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            return (R[0, 1] + R[1, 0]) / s, s / 4.0, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4.0, (R[1, 0] - R[0, 1]) / s

    def _publish_marker_tf(self, rvec, tvec, marker_id, frame_id):
        R_cv, _ = cv2.Rodrigues(rvec)
        R_ros = self.R_cv_to_ros @ R_cv
        t_ros = self.R_cv_to_ros @ tvec

        qx, qy, qz, qw = self._matrix_to_quat(R_ros)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = f'aruco_marker_{marker_id}'
        t.transform.translation.x = float(t_ros[0])
        t.transform.translation.y = float(t_ros[1])
        t.transform.translation.z = float(t_ros[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

        for tf_frame in (self.goal_frame, 'odom'):
            try:
                cam_tf = self.tf_buffer.lookup_transform(
                    tf_frame, frame_id,
                    self.get_clock().now(),
                    rclpy.time.Duration(seconds=0.05)
                )
            except Exception:
                continue
            R_target_to_cam = self._quat_to_matrix(
                cam_tf.transform.rotation.x, cam_tf.transform.rotation.y,
                cam_tf.transform.rotation.z, cam_tf.transform.rotation.w,
            )
            t_target_to_cam = np.array([
                cam_tf.transform.translation.x,
                cam_tf.transform.translation.y,
                cam_tf.transform.translation.z,
            ], dtype=np.float64)
            t_marker = R_target_to_cam @ t_ros + t_target_to_cam
            self.marker_positions[marker_id] = {
                'x': float(t_marker[0]),
                'y': float(t_marker[1]),
                'frame': tf_frame,
            }
            self.get_logger().info(
                f'Cached marker {marker_id} position: '
                f'({t_marker[0]:.2f}, {t_marker[1]:.2f}) in {tf_frame}'
            )
            break

    def _try_send_nav_goal(self):
        mid = self._target_marker_id
        if mid is None:
            self.state = 'SPINNING'
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            return

        if int(mid) in self.marker_positions:
            pos = self.marker_positions[int(mid)]
            self.get_logger().info(
                f'Sending Nav2 goal to marker {int(mid)} '
                f'({pos["x"]:.2f}, {pos["y"]:.2f}) in {pos["frame"]}'
            )
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = pos['frame']
            goal_msg.pose.pose.position.x = pos['x']
            goal_msg.pose.pose.position.y = pos['y']
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.w = 1.0
            self._send_goal(goal_msg, int(mid))
            return

        self.get_logger().info(
            f'Marker {int(mid)} detected, position not yet resolved...'
        )

    def _send_goal(self, goal_msg, marker_id):
        self.visited_markers.add(marker_id)
        self.get_logger().info(
            f'Sending Nav2 goal to marker {marker_id} '
            f'({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})'
        )
        send_future = self.nav_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)
        self._nav_goal_sent = True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Nav2 goal for marker {self._target_marker_id} was rejected')
            self._target_marker_id = None
            self.state = 'SPINNING'
            return

        self._goal_handle = goal_handle
        self.get_logger().info(f'Nav2 goal accepted, navigating to marker {self._target_marker_id}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        result = future.result()
        self._goal_handle = None
        self._nav_goal_sent = False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached marker {self._target_marker_id}! Waiting 2s...')
        else:
            self.get_logger().warn(
                f'Navigation to marker {self._target_marker_id} '
                f'failed with status {result.status}'
            )

        self.state = 'WAITING'
        self.wait_timer = self.create_timer(2.0, self._wait_done_callback)

    def _wait_done_callback(self):
        self.wait_timer.cancel()

        if len(self.visited_markers) >= self.total_markers:
            self.state = 'COMPLETE'
            self.get_logger().info(
                f'All {self.total_markers} markers visited! '
                f'Visited: {sorted(self.visited_markers)}'
            )
            return

        self.state = 'SPINNING'
        self._target_marker_id = None
        self.get_logger().info(
            f'Resuming search. Visited {len(self.visited_markers)}/{self.total_markers}: '
            f'{sorted(self.visited_markers)}'
        )

    def control_loop(self):
        if self.state == 'SPINNING':
            twist = Twist()
            twist.angular.z = self.spin_angular
            self.cmd_vel_pub.publish(twist)

        elif self.state == 'APPROACHING':
            elapsed = (self.get_clock().now() - self._approach_start).nanoseconds * 1e-9
            if elapsed < self.approach_duration:
                twist = Twist()
                twist.linear.x = self.approach_velocity
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'NAVIGATING'
                self._try_send_nav_goal()

        elif self.state == 'NAVIGATING' and not self._nav_goal_sent:
            twist = Twist()
            twist.angular.z = self.spin_angular
            self.cmd_vel_pub.publish(twist)
            self._try_send_nav_goal()

        elif self.state == 'COMPLETE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
