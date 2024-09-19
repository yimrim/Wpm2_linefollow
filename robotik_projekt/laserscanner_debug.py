"""
Simple driving node that is based on the driving behavior of a simple vacuum cleaner robot: The robot turns as long as
an obstacle is detected in the defined area, otherwise it drives straight ahead. To detect obstacles, only one measurement
value is used per scan of the laser scanner.
"""
import rclpy
import rclpy.node
from sensor_msgs.msg import LaserScan

class SimpleDriving(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('laserscan_beam_to_use', 0)

        # variable for the last sensor reading
        self.last_distance = 0.0

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

    # handling received laser scan data
    def scanner_callback(self, msg):

        # saving the required sensor value, no further processing at this point
        self.last_distance = msg.ranges[self.get_parameter('laserscan_beam_to_use').get_parameter_value().integer_value]
        print(self.last_distance)


def main(args=None):
    print('Hi from obstacle avoidance debug')
    rclpy.init(args=args)

    node = SimpleDriving()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
