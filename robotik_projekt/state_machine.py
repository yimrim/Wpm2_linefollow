import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from enum import Enum

class States(Enum):
    INIT = 0
    LINE_FOLLOWING = 1
    OBSTACLE = 2

class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_machine')

        self.state = States.INIT

        #local states
        self.stoplight_local = False
        self.obstacle_local = False

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for input decision data
        # stoplight subscriber
        self.stoplight_subscription = self.create_subscription(Bool, 'stoplight', self.stoplight_callback, qos_profile=qos_policy)
        # obstacle subscriber
        self.obstacle_subscription = self.create_subscription(Bool, '/obstacle_detector', self.obstacle_callback, qos_profile=qos_policy)
        # line_following node subscriber
        self.line_following_subscription = self.create_subscription(Twist, '/line_following_twist', self.line_following_callback, qos_profile=qos_policy)
        # obstacle_avoidance drive instruction subscriber
        self.obstacle_avoidance_subscription = self.create_subscription(Twist, '/obstacle_avoidance_twist', self.obstacle_avoidance_callback, qos_profile=qos_policy)

        # create publisher for driving commands
        self.driving_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # driving logic and state machine
    def timer_callback(self):
        #################################### State machine
        if self.state == States.INIT:
            print("INITIAL STATE")
            if self.stoplight_local == True:
                self.state = States.LINE_FOLLOWING
        elif self.state == States.LINE_FOLLOWING:
            if self.obstacle_local == True:
                self.state = States.OBSTACLE
            print("Line Following")
        elif self.state == States.OBSTACLE:
            if self.obstacle_local == False:
                self.state = States.LINE_FOLLOWING
            print("OBSTACLE")

    #################################### Callbacks

    def stoplight_callback(self, data):
        self.stoplight_local = data.data

    def line_following_callback(self, data):
        if self.state == States.LINE_FOLLOWING:
            self.driving_publisher.publish(data)

    def obstacle_callback(self, data):
        self.obstacle_local = data.data

    def obstacle_avoidance_callback(self, data):
        if self.state == States.OBSTACLE:
            self.driving_publisher.publish(data)

def main(args=None):
    print('Hi from robotik_projekt state_machine.')
    rclpy.init(args=args)

    node = StateMachine()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
