import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class SimpleController(Node):
    """
    A minimal state machine controller to arm the drone and switch to offboard mode.
    """

    # PX4 Arming States
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2

    def __init__(self):
        super().__init__('simple_controller')
        
        # PX4 uorb publishes over UDP which is inherently unreliable
        self.qos_profile_sensor_data = QoSPresetProfiles.SENSOR_DATA.value
        
        # Use a standard reliable profile to ensure commands arrive
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_publisher
        )
        
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_publisher
        )
        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_publisher
        )

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v1', 
            self.vehicle_status_callback, 
            self.qos_profile_sensor_data
        )

        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = self.ARMING_STATE_DISARMED  # Initialize using internal constant
        self.offboard_requested = False

        # 20 Hz (50ms) as required by PX4 for stability
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Target Setpoint (NED = North, East, Down)
        self.target_position = [0.0, 0.0, -5.0]
        self.target_yaw = 0.0

        self.get_logger().info('Initialized. Waiting for vehicle status')

    def vehicle_status_callback(self, msg: VehicleStatus):
        """
        Callback to update the internal state machine with vehicle telemetry.
        """
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def timer_callback(self):
        """
        Main control loop running at 20Hz.
        1. Publishes OffboardControlMode heartbeat.
        2. Publishes TrajectorySetpoint.
        3. Handles state transitions.
        """
        # Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True    # Sending position setpoints
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Publish TrajectorySetpoint
        traj_msg = TrajectorySetpoint()
        traj_msg.position = self.target_position
        traj_msg.yaw = self.target_yaw
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(traj_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Drone is in offboard mode no action needed other than holding setpoints
            pass
        else:
            # If not offboard try to arm first then switch to offboard
            if self.arming_state == self.ARMING_STATE_DISARMED:
                self.get_logger().info('Attempting to arm vehicle')
                self.arm_vehicle()
            elif self.arming_state == self.ARMING_STATE_ARMED:
                # Once armed request offboard mode
                if not self.offboard_requested:
                    self.get_logger().info('Armed, switching to offboard mode')
                    self.engaged_offboard_mode()
                    self.offboard_requested = True

    def arm_vehicle(self):
        """
        Sends a vehicle command to arm the motors.
        """
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # 1.0 Arm, 0.0 Disarm
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(cmd)

    def engaged_offboard_mode(self):
        """
        Sends a vehicle command to switch flight mode to offboard.
        """
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # Custom mode -main-
        cmd.param2 = 6.0  # Offboard -sub mode-
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    simple_controller = None
    
    try:
        simple_controller = SimpleController()
        rclpy.spin(simple_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Caught exception: {e}")
    finally:
        if simple_controller:
            simple_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()