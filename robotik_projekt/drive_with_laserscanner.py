"""
Simple driving node that is based on the driving behavior of a simple vacuum cleaner robot: The robot turns as long as
an obstacle is detected in the defined area, otherwise it drives straight ahead. To detect obstacles, only one measurement
value is used per scan of the laser scanner.
"""
import time

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum

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

class SimpleDriving(rclpy.node.Node):

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

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # handling received laser scan data
    def scanner_callback(self, msg):

        # saving the required sensor value, no further processing at this point
        self.last_distance = msg.ranges[self.get_parameter('laserscan_beam_to_use').get_parameter_value().integer_value]
        print(self.last_distance)

    # driving logic
    def timer_callback(self):

        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        timeout_turn = self.get_parameter('timeout_turn').get_parameter_value().double_value
        timeout_drive = self.get_parameter('timeout_drive').get_parameter_value().double_value

        print(self.obstacle_state)


        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.y = 0.0
        if self.obstacle_state == AvoidanceStates.NO_OBSTACLE:

            if self.last_distance > distance_stop:
                msg.linear.x = speed_drive
                self.drive_publisher.publish(msg)
            else:
                msg.linear.x = 0.0
                self.drive_publisher.publish(msg)
                self.obstacle_state = AvoidanceStates.OBSTACLE_IN_FRONT
        elif self.obstacle_state == AvoidanceStates.OBSTACLE_IN_FRONT:
            msg.angular.z = speed_turn
            self.drive_publisher.publish(msg)
            time.sleep(timeout_turn)
            msg.angular.z = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.DRIVING_FORWARD
        elif self.obstacle_state == AvoidanceStates.DRIVING_FORWARD:
            msg.linear.x = speed_drive
            self.drive_publisher.publish(msg)
            time.sleep(timeout_drive)
            msg.linear.x = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.TURNING_RIGHT
        elif self.obstacle_state == AvoidanceStates.TURNING_RIGHT:
            msg.angular.z = -speed_turn
            self.drive_publisher.publish(msg)
            time.sleep(timeout_turn)
            msg.angular.z = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.PASS_OBSTACLE
        elif self.obstacle_state == AvoidanceStates.PASS_OBSTACLE:
            msg.linear.x = speed_drive
            self.drive_publisher.publish(msg)
            time.sleep(2 * timeout_drive)
            msg.linear.x = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.REARRANGE_RIGHT
        elif self.obstacle_state == AvoidanceStates.REARRANGE_RIGHT:
            msg.angular.z = -speed_turn
            self.drive_publisher.publish(msg)
            time.sleep(timeout_turn)
            msg.angular.z = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.REARRANGE_IN_FRONT
        elif self.obstacle_state == AvoidanceStates.REARRANGE_IN_FRONT:
            msg.linear.x = speed_drive
            self.drive_publisher.publish(msg)
            time.sleep(timeout_drive)
            msg.linear.x = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.REARRANGE_LEFT
        elif self.obstacle_state == AvoidanceStates.REARRANGE_LEFT:
            msg.angular.z = speed_turn
            self.drive_publisher.publish(msg)
            time.sleep(timeout_turn)
            msg.angular.z = 0.0
            self.drive_publisher.publish(msg)
            self.obstacle_state = AvoidanceStates.NO_OBSTACLE


def main(args=None):
    print('Hi from obstacle avoidance')
    rclpy.init(args=args)

    node = SimpleDriving()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
