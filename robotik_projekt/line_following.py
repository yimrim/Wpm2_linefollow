"""
simple line following node
"""

import cv2
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage


class LineFollowing(rclpy.node.Node):

    def __init__(self):
        super().__init__('line_following')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 500)
        self.declare_parameter('boundary_right', 600)
        self.declare_parameter('speed_drive', -0.05)
        self.declare_parameter('speed_turn', 0.5)
        self.declare_parameter('height_offset', 50)
        self.declare_parameter('left_image_cut', 320)
        self.declare_parameter('right_image_cut', 5)

        # position of brightest pixel in
        self.lineposition = 640 / 2

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'line_following_twist', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # handling received image data
    def scanner_callback(self, data):
        # caching the parameters for reasons of clarity
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        height_offset = self.get_parameter('height_offset').get_parameter_value().integer_value
        left_image_cut = self.get_parameter('left_image_cut').get_parameter_value().integer_value
        right_image_cut = self.get_parameter('right_image_cut').get_parameter_value().integer_value


        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]

        #helping points
        offset = height-height_offset
        point1 = (boundary_left, offset)
        point2 = (boundary_right, offset)

        #circle props
        radius = 5
        color = (255,0,0)
        thickness = 2

        # get the lowest row from image
        img_row = img_gray[height - height_offset, :]

        self.lineposition = width / 2
        brightness = 0
        for x in range(len(img_row)):
            if x > left_image_cut and x < 640 - right_image_cut:
                if img_row[x] >= brightness + 5:
                    brightness = img_row[x]
                    # print("index: " + str(x) + " brightness: " + str(brightness))
                    self.lineposition = x
        # print(self.lineposition)

        cv2.circle(img_gray, point1, radius, color, thickness)
        cv2.circle(img_gray, point2, radius, color, thickness)

        cv2.circle(img_gray, (self.lineposition, offset), 2 * radius, color, thickness)

        # show image
        cv2.imshow("IMG", img_gray)
        cv2.imshow("IMG_ROW", img_row)
        cv2.waitKey(1)

    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value

        if (self.lineposition > boundary_right):
            self.drive('right')
        elif self.lineposition < boundary_left:
            self.drive('left')
        else:
            self.drive('forward')



    def drive(self, direction):
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        msg = Twist()
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed = speed_drive
        turn = 0.0  # default linie mittig

        if direction == 'left':
            turn = speed_turn * 1
            speed = 0.0
        elif direction == 'right':
            turn = speed_turn * -1
            speed = 0.0
        elif direction == 'forward':
            turn = 0.00
            speed = speed_drive * 1
        
        print(direction)

        msg.linear.x = speed
        msg.angular.z = turn
        self.publisher_.publish(msg)


def main(args=None):
    print('Hi from robotik_projekt line following.')
    rclpy.init(args=args)

    node = LineFollowing()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
