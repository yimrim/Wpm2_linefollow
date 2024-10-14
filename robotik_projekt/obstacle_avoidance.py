import time
from enum import Enum

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class AvoidanceStates(Enum):
    NO_OBSTACLE = 0,
    OBSTACLE_IN_FRONT = 1,
    TURNING_LEFT = 2,
    OBSTACLE_IN_RIGHT = 3
    DRIVING_FORWARD = 4,
    TURNING_RIGHT = 5,
    PASS_OBSTACLE = 6,
    REARRANGE_RIGHT = 7,
    REARRANGE_IN_FRONT = 8,
    REARRANGE_LEFT = 9,


class ObstacleAvoidance(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('speed_turn', 0.4)
        self.declare_parameter('speed_drive', -0.05)
        self.declare_parameter('timeout_turn', 4.0)
        self.declare_parameter('timeout_drive', 5.0)
        self.declare_parameter('laserscan_beam_to_use', 0)

        # variable for the last sensor reading
        self.last_distance = 100.0

        # initialize state
        self.obstacle_state = AvoidanceStates.NO_OBSTACLE

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # publishers
        self.drive_publisher = self.create_publisher(Twist, '/obstacle_avoidance_twist', 1)
        self.obstacle_detector_publisher = self.create_publisher(Bool, '/obstacle_detector', False)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    def scanner_callback(self, msg):
        beam_to_use = self.get_parameter('laserscan_beam_to_use').get_parameter_value().integer_value
        self.last_distance = msg.ranges[beam_to_use]

    def publish_drive_command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.drive_publisher.publish(msg)

    def publish_obstacle_detector(self, detected):
        bool_msg = Bool()
        bool_msg.data = detected
        self.obstacle_detector_publisher.publish(bool_msg)

    def sleep_and_stop(self, timeout):
        time.sleep(timeout)
        self.publish_drive_command(0.0, 0.0)

    def timer_callback(self):
        # Retrieve parameters
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        timeout_turn = self.get_parameter('timeout_turn').get_parameter_value().double_value
        timeout_drive = self.get_parameter('timeout_drive').get_parameter_value().double_value

        # Obstacle avoidance logic
        if self.obstacle_state == AvoidanceStates.NO_OBSTACLE:
            if self.last_distance > distance_stop:
                self.publish_obstacle_detector(False)
            else:
                self.publish_obstacle_detector(True)
                self.publish_drive_command(0.0, 0.0)
                self.obstacle_state = AvoidanceStates.OBSTACLE_IN_FRONT

        elif self.obstacle_state == AvoidanceStates.OBSTACLE_IN_FRONT:
            self.publish_drive_command(angular=speed_turn)
            self.sleep_and_stop(timeout_turn)
            self.obstacle_state = AvoidanceStates.DRIVING_FORWARD

        elif self.obstacle_state == AvoidanceStates.DRIVING_FORWARD:
            self.publish_drive_command(linear=speed_drive)
            self.sleep_and_stop(timeout_drive)
            self.obstacle_state = AvoidanceStates.TURNING_RIGHT

        elif self.obstacle_state == AvoidanceStates.TURNING_RIGHT:
            self.publish_drive_command(angular=-speed_turn)
            self.sleep_and_stop(timeout_turn)
            self.obstacle_state = AvoidanceStates.PASS_OBSTACLE

        elif self.obstacle_state == AvoidanceStates.PASS_OBSTACLE:
            self.publish_drive_command(linear=speed_drive)
            self.sleep_and_stop(2 * timeout_drive)
            self.obstacle_state = AvoidanceStates.REARRANGE_RIGHT

        elif self.obstacle_state == AvoidanceStates.REARRANGE_RIGHT:
            self.publish_drive_command(angular=-speed_turn)
            self.sleep_and_stop(timeout_turn)
            self.obstacle_state = AvoidanceStates.REARRANGE_IN_FRONT

        elif self.obstacle_state == AvoidanceStates.REARRANGE_IN_FRONT:
            self.publish_drive_command(linear=speed_drive)
            self.sleep_and_stop(timeout_drive)
            self.obstacle_state = AvoidanceStates.REARRANGE_LEFT

        elif self.obstacle_state == AvoidanceStates.REARRANGE_LEFT:
            self.publish_drive_command(angular=speed_turn)
            self.sleep_and_stop(timeout_turn)
            self.obstacle_state = AvoidanceStates.NO_OBSTACLE


def main(args=None):
    print('Hi from obstacle avoidance')
    rclpy.init(args=args)

    node = ObstacleAvoidance()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
