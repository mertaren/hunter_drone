import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.force_pub = self.create_publisher(
            Wrench,
            '/hunter/cmd_force',
            10
        )

        # Control loop rate
        self.timer_period = 0.01 # For flight control
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info('Taking off...')

    def control_loop(self):
        
        msg = Wrench()

        # Physcis calculation

        msg.force.x = 0.0
        msg.force.y = 0.0
        msg.force.z = 17.0

        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        self.force_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()