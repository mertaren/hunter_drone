import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

# ROS 2 Messages
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

# Constans
MAX_DEPTH_AGE_SEC   = 0.50   # Max allowed age for a depth frame
MAX_DETECT_AGE_SEC  = 0.10   # Max allowed age for a detection
DEPTH_ROI_HALF      = 4      # ROI: pixels around the center
DEPTH_MIN_M         = 0.2    # Min valid depth (meters)
DEPTH_MAX_M         = 50.0   # Max valid depth (meters)
CHI2_3DOF_99        = 11.34  # Chi-square threshold for 3 DOF at 99% confidence


class EKFTracker3D(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        # Parameters
        self.declare_parameter('sigma_a',       2.0)   # Process noise 
        self.declare_parameter('meas_std_base', 0.3)   # Base measurement noise
        self.declare_parameter('meas_std_ref_dist', 5.0)  # Reference distance for the base noise

        # Forward-facing camera: [0, 0, 0]
        # Downward-facing camera: [0, 90, 0]
        self.declare_parameter('cam_to_body_rpy_deg', [0.0, 0.0, 0.0])

        self.sigma_a           = self.get_parameter('sigma_a').value
        self.meas_std_base     = self.get_parameter('meas_std_base').value
        self.meas_std_ref_dist = self.get_parameter('meas_std_ref_dist').value
        cam_rpy_deg            = self.get_parameter('cam_to_body_rpy_deg').value

        self._R_cam_to_body = R.from_euler('xyz', cam_rpy_deg, degrees=True).as_matrix()

        # State: [X, Y, Z, Vx, Vy, Vz] in Global NED
        self.state      = np.zeros(6)
        self.covariance = np.eye(6) * 1000.0

        # Drone state cache
        self.drone_pos  = np.zeros(3)
        self.drone_quat = np.array([0.0, 0.0, 0.0, 1.0])

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = 0.0

        self.latest_depth_image    = None
        self.latest_depth_stamp_ns = 0   # Nanoseconds

        # System flags / lock
        self.is_odom_ready     = False
        self.is_cam_info_ready = False
        self.is_initialized    = False
        self._lock             = threading.Lock()

        # ROS2 comm
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_best_effort, callback_group=self.cb_group)

        self.create_subscription(
            Detection2DArray, '/vision/detections',
            self.measurement_callback, 10, callback_group=self.cb_group)

        self.cam_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.cam_info_callback, 10, callback_group=self.cb_group)

        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback, qos_best_effort, callback_group=self.cb_group)

        # Publishers
        self.target_pub      = self.create_publisher(PointStamped, '/target/global_position', 10)
        self.target_odom_pub = self.create_publisher(Odometry, '/target/odometry', 10)

        # Timer
        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / 30.0, self.prediction_step, callback_group=self.cb_group)

        self.get_logger().info(
            "Hunter EKF 3D Tracker Initialized\n"
            f"  cam_to_body_rpy_deg = {cam_rpy_deg}\n"
            f"  meas_std_base = {self.meas_std_base} m @ {self.meas_std_ref_dist} m ref dist"
        )

    # Callbacks
    def cam_info_callback(self, msg):
        with self._lock:
            if not self.is_cam_info_ready:
                self.fx = msg.k[0]
                self.cx = msg.k[2]
                self.fy = msg.k[4]
                self.cy = msg.k[5]
                self.is_cam_info_ready = True
                self.destroy_subscription(self.cam_sub)
                self.get_logger().info(
                    f"Camera Intrinsics Locked: fx={self.fx:.1f}, fy={self.fy:.1f}, "
                    f"cx={self.cx:.1f}, cy={self.cy:.1f}"
                )

    def odom_callback(self, msg):
        with self._lock:
            self.drone_pos  = np.array([msg.position[0], msg.position[1], msg.position[2]])
            self.drone_quat = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])  # [x, y, z, w]
            self.is_odom_ready = True

    def depth_callback(self, msg):
        """Caches the depth image along with its precise timestamp."""
        with self._lock:
            if msg.encoding == '32FC1':
                img = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            elif msg.encoding == '16UC1':
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)) / 1000.0
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}", throttle_duration_sec=5.0)
                return

            self.latest_depth_image    = img
            self.latest_depth_stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    # EKF prediction update
    def prediction_step(self):
        with self._lock:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # Keep dt within realistic bounds to prevent covariance explosion
            dt = float(np.clip(dt, 0.001, 0.1))

            # State Transition Matrix - F
            F = np.eye(6)
            F[0, 3] = dt
            F[1, 4] = dt
            F[2, 5] = dt

            # Process Noise Matrix - Q 
            var_a = self.sigma_a ** 2
            dt2 = dt ** 2
            dt3 = dt ** 3
            dt4 = dt ** 4

            Q = np.zeros((6, 6))
            Q[0, 0] = Q[1, 1] = Q[2, 2] = 0.25 * dt4 * var_a
            Q[3, 3] = Q[4, 4] = Q[5, 5] = dt2 * var_a
            Q[0, 3] = Q[3, 0] = Q[1, 4] = Q[4, 1] = Q[2, 5] = Q[5, 2] = 0.5 * dt3 * var_a

            self.state      = F @ self.state
            self.covariance = F @ self.covariance @ F.T + Q

            self.publish_result()

    # EKF mueasurement update
    def measurement_callback(self, msg):
        with self._lock:
            # Precondition checks
            if not self.is_odom_ready or not self.is_cam_info_ready:
                return
            if self.latest_depth_image is None or len(msg.detections) == 0:
                return

            # Staleness check
            detect_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            depth_age_sec = abs(detect_ns - self.latest_depth_stamp_ns) / 1e9
            if depth_age_sec > MAX_DEPTH_AGE_SEC:
                self.get_logger().warn(
                    f"Depth frame too old ({depth_age_sec*1000:.0f} ms) skipping",
                    throttle_duration_sec=1.0
                )
                return

            # Select the detection with the highest confidence score
            best = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
            u = int(best.bbox.center.position.x)
            v = int(best.bbox.center.position.y)

            # ROI
            h, w = self.latest_depth_image.shape
            u_c  = int(np.clip(u, 0, w - 1))
            v_c  = int(np.clip(v, 0, h - 1))

            u0 = max(0, u_c - DEPTH_ROI_HALF)
            u1 = min(w,  u_c + DEPTH_ROI_HALF + 1)
            v0 = max(0, v_c - DEPTH_ROI_HALF)
            v1 = min(h,  v_c + DEPTH_ROI_HALF + 1)

            roi   = self.latest_depth_image[v0:v1, u0:u1]
            valid = roi[np.isfinite(roi) & (roi >= DEPTH_MIN_M) & (roi <= DEPTH_MAX_M)]

            if valid.size < 3:
                self.get_logger().debug("Invalid depth ROI skipping measurement")
                return

            depth = float(np.median(valid))

            # 3D global point
            z_meas = self._get_3d_point_from_depth(u_c, v_c, depth)

            if not self.is_initialized:
                self.state[:3]  = z_meas
                self.state[3:]  = 0.0
                self.covariance = np.eye(6) * 5.0
                self.is_initialized = True
                self.get_logger().info(
                    f"Filter Initialized at: X={z_meas[0]:.2f}, "
                    f"Y={z_meas[1]:.2f}, Z={z_meas[2]:.2f}"
                )
                return

            # Observation Matrix
            H = np.zeros((3, 6))
            H[0, 0] = H[1, 1] = H[2, 2] = 1.0

            # Innovation
            y      = z_meas - self.state[:3]

            # Dynamic measurement covariance
            dyn_std = self.meas_std_base * (depth / self.meas_std_ref_dist)
            dyn_std = max(dyn_std, self.meas_std_base)
            R_cov   = np.eye(3) * (dyn_std ** 2)

            # Innovation covariance and outlier rejection
            S = H @ self.covariance @ H.T + R_cov

            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                self.get_logger().error("S matrix is singular skipping measurement")
                return

            mahalanobis = float(y.T @ S_inv @ y)
            if mahalanobis > CHI2_3DOF_99:
                self.get_logger().warn(
                    f"3D Outlier rejected Mahalanobis Dist: {mahalanobis:.2f}",
                    throttle_duration_sec=0.5
                )
                return

            # Kalman Gain and State Update
            K = self.covariance @ H.T @ S_inv
            self.state = self.state + K @ y

            # Covariance Update (Joseph form)
            I_KH            = np.eye(6) - K @ H
            self.covariance = I_KH @ self.covariance @ I_KH.T + K @ R_cov @ K.T

    def _get_3d_point_from_depth(self, u: int, v: int, depth: float) -> np.ndarray:
        """
        Converts Pixel (u,v) + depth into a Global NED 3D point.

        Coordinate Frames:
            Camera Optical : Z forward, X right, Y down
            Body (NED)     : X forward, Y right, Z down
        """
        # Pixel -> Camera optical frame
        x_cam = (u - self.cx) * depth / self.fx
        y_cam = (v - self.cy) * depth / self.fy
        z_cam = depth

        # Camera optical -> Camera NED Frame
        p_cam_ned = np.array([z_cam, x_cam, y_cam])

        # Camera NED -> Body NED
        p_body = self._R_cam_to_body @ p_cam_ned

        # Body NED -> Global NED
        r_drone = R.from_quat(self.drone_quat)   # [x, y, z, w]
        p_global = self.drone_pos + r_drone.apply(p_body)

        return p_global

    def publish_result(self):
        """Publishes the estimated state only if the filter has been initialized."""
        if not self.is_initialized:
            return

        stamp = self.get_clock().now().to_msg()

        # Position
        pt_msg                 = PointStamped()
        pt_msg.header.stamp    = stamp
        pt_msg.header.frame_id = "map"
        pt_msg.point.x = float(self.state[0])
        pt_msg.point.y = float(self.state[1])
        pt_msg.point.z = float(self.state[2])
        self.target_pub.publish(pt_msg)

        # Odometry
        odom_msg                          = Odometry()
        odom_msg.header                   = pt_msg.header
        odom_msg.child_frame_id           = "target_estimated"
        odom_msg.pose.pose.position       = pt_msg.point
        odom_msg.twist.twist.linear.x     = float(self.state[3])
        odom_msg.twist.twist.linear.y     = float(self.state[4])
        odom_msg.twist.twist.linear.z     = float(self.state[5])
        self.target_odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFTracker3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()