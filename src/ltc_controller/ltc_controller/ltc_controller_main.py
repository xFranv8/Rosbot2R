import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2

from ServerConnection import ServerConnection

import time


class LTCController(Node):
    def __init__(self):
        super().__init__('ltc_controller')

        self.im_sub = self.create_subscription(CompressedImage, 'camera/color/image_raw/compressed', 
                                               self.im_listener_callback, 
                                               qos_profile=qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)
        self.timer = self.create_timer(0.05, self.control_cycle)

        self.bridge = CvBridge()

        self.last_image = None

        self.connection: ServerConnection = ServerConnection("10.0.77.20", 5009)

        self.t0 = time.time()
    
    def im_listener_callback(self, msg):
        self.last_image = self.bridge.compressed_imgmsg_to_cv2(msg)

    def control_cycle(self):
        if self.last_image is None:
            return
        
        output_vel = Twist()

        t0 = time.time()

        v, w = self.connection.send_image(self.last_image)

        t1 = time.time()

        print(f"FPS: {1 / (t1 - t0)}")

        output_vel.linear.x = v
        output_vel.angular.w = w

        self.vel_pub.publish(output_vel)


def main(args=None):
    rclpy.init(args=args)

    ltc_controller_node = LTCController()

    rclpy.spin(ltc_controller_node)

    ltc_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
