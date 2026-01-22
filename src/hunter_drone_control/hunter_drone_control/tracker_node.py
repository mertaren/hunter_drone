import time
import numpy as np
from typing import Optional # Type hint

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.timer import Timer

from px4_msgs.msg import(
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus
)
from vision_msgs.msg import Detection2DArray

class VisualTracker(Node):
    """
    ROS 2 node for autonomous visual tracking

    Subs to:
        - /vision/detections
        - ~/.../vehicle_status
    Pub to:
        - ~/.../trajectory_setpoint
        - ~/.../offboard_control_mode
    """
    def __init__(self) -> None:
        super().__init__('visual_tracker')

        # Image res
        self.declare_parameter('img_width', 640)
        self.declare_parameter('img_height', 480)

        # Control gains
        self.declare_parameter('kp_yaw', 0.002)
        self.declare_parameter('kp_z', 0.005)

        # Pixel margin to ignore small errors
        self.declare_parameter('deadzone_x', 20)

        # Wait before declaring target loss
        self.declare_parameter('target_timeout', 2.0)

        # Safety interlock
        self.declare_parameter('enable_control', False) # Must be set True

        # LOAD PARAMS
        self.img_width = self.get_parameter('img_width').value
        self.img_height = self.get_parameter('img_height').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kp_z = self.get_parameter('kp_z').value
        self.deadzone_x = self.get_parameter('deadzone_x').value
        self.target_timeout = self.get_parameter('target_timeout').value
        self.enable_control = self.get_parameter('enable_control').value

        # State variables
        self.target_center_x: Optional[float] = None
        self.target_center_y: Optional[float] = None
        self.last_detection_time: float = 0.0
        self.target_visible: bool = False
        self.nav_state: int = 0  
        self.arming_state: int = 0

        # Find image center (target setpoint s*)
        self.cx = self.img_width / 2.0
        self.cy = self.img_height / 2.0

        # QOS CONFIGURATION

        # Sensor data: Best Effort - drop old data
        qos_sensor = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Control data: Volatile for streaming setpoints to avoid buffer lag
        qos_control = qos_sensor(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PUB / SUB
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_control
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_control
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_control
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray, '/vision/detections', self.detection_callback, qos_sensor
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_sensor
        )


        # Main control loop 
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Tracker node initialized 'READY!")
        self.get_logger().warn("Set 'enable_control' to True to engage!")

    # Callbacks
    def detection_callback(self, msg: Detection2DArray) -> None:
        """
        Called when YOLO publish new detection
        Updates the internal state estimate of the target position
        """
        if not msg.detections:
            return
        try:
            detection = msg.detections[0]
            self.target_center_x = detection.bbox.center.position.x
            self.target_center_y = detection.bbox.center.position.y
            self.last_detection_time = time.time()

            if not self.target_visible:
                self.target_visible = True
                self.get_logger().info("target acquired")
        except IndexError:
            pass
    
    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        """
        Monitor arming state
        """
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
    
    # Main control logic
    def control_loop(self) -> None:
        """
        Executed at 20hz
        - Check params
        - Send heartbeat
        - Calculate p-control
        - Publish velocity
        """

        try:
            self.enable_control = self.get_parameter("enable_control").value
        except:
            pass

        self.publish_offboard_mode() # px4 hearbeat

        # Safety
        if not self.enable_control:
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
            return
        
        # Target visibilty
        current_time = time.time()
        if(current_time - self.last_detection_time) > self.target_timeout:
            if self.target_visible:
                self.target_visible = False
                self.get_logger().warn("Target lost")
            
            # Stop all movement
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)

        # Validity check    
        if self.target_center_x is None or self.target_center_y is None:
            return

        # P-Control

        # Yaw control
        # Direct relatioship +Error -> +Control
        error_x = self.target_center_x - self.cx

        if abs(error_x) > self.deadzone_x:
            yaw_speed = self.kp_yaw * error_x
        else:
            yaw_speed = 0.0
        
        # Altitude control
        # Inverse relationship +Error -> -Control
        error_y = self.cy - self.target_center_y
        velocity_z = -(self.kp_z * error_y)

        # Limit yaw rate to +- 0.5 rad/s
        yaw_speed = np.clip(yaw_speed, -0.5, 0.5)

        # Limit vertical speed
        velocity_z = np.clip(velocity_z, -0.5, 0.5)

        self.publish_trajectory_setpoint(
            vx=0.0,
            vy=0.0,
            yz=velocity_z,
            yawspeed= yaw_speed
        )


    def publish_offboard_mode(self) -> None:
        """
        Publishes OffboardControlMode to notify PX4 of incoming velocity commands
        """
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True   # Enable Velocity Control
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float) -> None:
        """
        Publishes the calculated control inputs to the flight controller
        """
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')] # Position control disabled
        msg.velocity = [vx, vy, vz]
        msg.yaw = float('nan')
        msg.yawspeed = yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    tracker = VisualTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()