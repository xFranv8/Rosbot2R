import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

from ServerConnection import ServerConnection


class LTCController(Node):
    def __init__(self):
        super().__init__('ltc_controller')

        self.im_sub = self.create_subscription(Image, 'input_image', self.im_listener_callback, qos_profile_sensor_data)

        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)
        self.timer = self.create_timer(0.05, self.control_cycle)

        self.bridge = CvBridge()

        self.last_image: cv2.Mat = None

        self.connection: ServerConnection = ServerConnection("capodgxstation.etsii.urjc.es", 50003)
    
    def im_listener_callback(self, msg):
        self.last_image = self.bridge.compressed_imgmsg_to_cv2(msg)

    def control_cycle(self):
        output_vel = Twist()

        v, w = self.connection.send_image(self.last_image)

        output_vel.linear.x = v
        output_vel.angular.w = w

        self.vel_pub.publish(output_vel)

