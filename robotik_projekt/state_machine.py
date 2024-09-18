import rclpy
import rclpy.node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

from states import States


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_machine')

        self.state = States.INIT

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for input decision data #todo stoplightstate, line_follow_twist, obstacle_state
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # driving logic and state machine
    def timer_callback(self):
        #################################### State machine todo legal state switches
        if self.state == States.INIT:
            #todo check if stoplight is green
            print("test")
        elif self.state == States.LINE_FOLLOWING:
            #todo check if obstacle is detected or not, forward driving command
            print("test2")
        elif self.state == States.OBSTACLE:
            #todo obstacle routine
            print("test3")



        ####################################

        # send driving message #todo
        # create message
        #print("test3")
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        self.publisher_.publish(msg)



def main(args=None):
    print('Hi from robotik_projekt state_machine.')
    rclpy.init(args=args)

    node = StateMachine()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
