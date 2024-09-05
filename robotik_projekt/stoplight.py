import cv2
import numpy as np
import rclpy
import rclpy.node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class Stoplight(rclpy.node.Node):

    def __init__(self):
        super().__init__('stoplight')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('lower_hue', 80)
        self.declare_parameter('upper_hue', 90)
        self.declare_parameter('lower_saturation', 100)
        self.declare_parameter('upper_saturation', 255)
        self.declare_parameter('lower_value', 200)
        self.declare_parameter('upper_value', 255)

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

    # handling received image data
    def scanner_callback(self, data):
        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # convert image to hsv
        hsv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only hue colors
        l_hue = self.get_parameter('lower_hue').value
        u_hue = self.get_parameter('upper_hue').value
        l_sat = self.get_parameter('lower_saturation').value
        u_sat = self.get_parameter('upper_saturation').value
        l_val = self.get_parameter('lower_value').value
        u_val = self.get_parameter('upper_value').value

        lower = np.array([l_hue, l_sat, l_val])
        upper = np.array([u_hue, u_sat, u_val])

        mask = cv2.inRange(hsv, lower, upper)
        print("Pixelcount: " + str(cv2.countNonZero(mask)))

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img_cv, img_cv, mask=mask)

        # Display the frame
        cv2.imshow('frame', img_cv)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)

        cv2.waitKey(1)

def main(args=None):
    print('Hi from robotik_projekt line following.')
    rclpy.init(args=args)

    node = Stoplight()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
