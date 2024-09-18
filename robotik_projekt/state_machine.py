import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from states import States


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_machine')

        self.state = States.INIT

        #local states
        self.stoplight_local = False

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for input decision data #todo line_follow_twist, obstacle_state
        # stoplight subscriber
        self.stoplight_subscription = self.create_subscription(Bool, 'stoplight', self.stoplight_callback, qos_policy=qos_policy)
        # line_following node subscriber
        self.line_following_subscription = self.create_subscription(Twist, 'line_following_twist', )

        # create publisher for driving commands
        self.driving_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # driving logic and state machine
    def timer_callback(self):
        #################################### State machine todo legal state switches
        if self.state == States.INIT:
            print("INITIAL STATE")
            if self.stoplight_local == True:
                self.state = States.LINE_FOLLOWING
        elif self.state == States.LINE_FOLLOWING:

            #todo check if obstacle is detected or not, forward driving command
            print("Line Following")
        elif self.state == States.OBSTACLE:
            #todo obstacle routine
            print("OBSTACLE")

    def stoplight_callback(self, data):
        self.stoplight_local = data.data

    def line_following_callback(self, data):
        if self.state == States.LINE_FOLLOWING:
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
