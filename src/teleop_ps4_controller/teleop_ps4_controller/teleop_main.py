import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

import pygame


class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_ps4_controller')

        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)
        self.timer = self.create_timer(0.05, self.control_cycle)

        # Initialize Pygame
        pygame.init()

        # Initialize the joystick module
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick found")
            return
        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print(f"Controller: {self.controller.get_name()}")
    
    def control_cycle(self):
        out_vel = Twist()

        pygame.event.pump()
        x_axis_l = self.controller.get_axis(0)
        y_axis_r = -self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT)

        out_vel.linear.x = x_axis_l
        out_vel.angular.z = y_axis_r

        self.vel_pub.publish(out_vel)


def main(args=None):
    rclpy.init(args=args)

    teleop_node = TeleopController()

    rclpy.spin(teleop_node)

    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    