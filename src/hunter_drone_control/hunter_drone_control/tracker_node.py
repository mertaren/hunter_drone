import math
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from nav_msgs.msg import Odometry

# Helper functions
def get_yaw_from_q(q):
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))

def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class TargetTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('visual_tracker')

        # Parameters
        self.declare_parameter('enable_control', False)       
        
        # Control gains
        self.declare_parameter('kp_xy', 0.4)                  # Forward/lateral speed gain
        self.declare_parameter('kp_z', 1.0)                   # Vertical speed gain
        self.declare_parameter('kp_yaw', 0.5)                 # Yaw rotation gain
        self.declare_parameter('max_speed', 3.0)              # Maximum speed limit (m/s)

        # Load parameters into variables
        self.enable_control = self.get_parameter('enable_control').value
        self.kp_xy          = self.get_parameter('kp_xy').value
        self.kp_z           = self.get_parameter('kp_z').value
        self.kp_yaw         = self.get_parameter('kp_yaw').value
        self.max_speed      = self.get_parameter('max_speed').value

        # State variables
        self.drone_pos = np.zeros(3)
        self.drone_yaw = 0.0
        self.nav_state = 0
        self.arming_state = 0

        self.target_pos: Optional[np.ndarray] = None
        self.last_target_time = self.get_clock().now()
        self.target_timeout_sec = 2.0  # Stop if target is lost for 2 seconds

        # ROS2 Communation
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_sensor)
        self.trajectory_pub    = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_sensor)
        self.vehicle_cmd_pub   = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_sensor)

        # Subscribers
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_sensor)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.drone_odom_cb, qos_sensor)
        self.create_subscription(Odometry, '/target/odometry', self.target_odom_cb, 10)

        # Main control loop running at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.heartbeat_counter = 0

        self.get_logger().info("Tracker node started. Waiting for EKF data.")
        self.get_logger().info("Set 'enable_control' parameter to true to start moving.")

    # Callback Functions
    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def drone_odom_cb(self, msg: VehicleOdometry):
        self.drone_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.drone_yaw = get_yaw_from_q(msg.q)

    def target_odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if any(math.isnan(v) or math.isinf(v) for v in [x, y, z]):
            self.get_logger().warn("Invalid position data received from EKF skipping this data.")
            return

        self.target_pos = np.array([x, y, z])
        self.last_target_time = self.get_clock().now()

    # Control Loop
    def control_loop(self):
        self.enable_control = self.get_parameter("enable_control").value

        # Heartbeat to keep PX4 in offboard mode
        self.publish_offboard_mode()

        # If control is disabled keep the drone in place
        if not self.enable_control:
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Arming and offboard mode transitions
        if self.heartbeat_counter % 10 == 0:
            if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Arming drone...")
                self.arm()
            elif self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("Switching to offboard mode...")
                self.engage_offboard_mode()
        self.heartbeat_counter += 1

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
            return

        # Failsafe
        time_since_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if self.target_pos is None or time_since_target > self.target_timeout_sec:
            if self.heartbeat_counter % 20 == 0:
                self.get_logger().warn("Target lost or too old. Hovering...")
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
            return
        
        # Distance vectors between drone and target
        dx = self.target_pos[0] - self.drone_pos[0]
        dy = self.target_pos[1] - self.drone_pos[1]
        
        dist_xy = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # YAW Control
        yaw_error = wrap_pi(target_angle - self.drone_yaw)
        v_yaw = self.kp_yaw * yaw_error
        v_yaw = np.clip(v_yaw, -1.5, 1.5)
        
        speed_xy = self.kp_xy * dist_xy
        speed_xy = np.clip(speed_xy, -self.max_speed, self.max_speed)
        
        # Split calculated speed into world frame components
        v_x = speed_xy * math.cos(target_angle)
        v_y = speed_xy * math.sin(target_angle)

        # Altitude Control
        desired_z = self.target_pos[2] - 1.0
        z_error = desired_z - self.drone_pos[2]
        
        v_z = self.kp_z * z_error
        v_z = np.clip(v_z, -2.0, 2.0) 

        # Send all calculated speeds to PX4
        self.publish_trajectory_setpoint(v_x, v_y, v_z, v_yaw)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0, 
            param2=6.0  
        )

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = TargetTrackerNode()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()