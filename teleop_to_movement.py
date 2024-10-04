#! /usr/bin/env python3

import sys
import threading
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        # Подписываемся на топик /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Публикуем в топики управления джойнтами
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.drive_publisher = self.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)

    def cmd_vel_callback(self, msg):
        # Извлекаем линейную и угловую скорость
        linear_velocity = msg.linear.x  # Скорость движения вперед/назад
        angular_velocity = msg.angular.z  # Скорость поворота

        # Публикуем команды для управления джойнтами

        steering_msg = Float64MultiArray()
        steering_msg.data = [angular_velocity]

        drive_msg = Float64MultiArray()
        drive_msg.data = [linear_velocity]  # Движение мотор-колеса
        print(drive_msg.data, steering_msg.data)

        self.steering_publisher.publish(steering_msg)
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_controller = TeleopController()
    rclpy.spin(teleop_controller)
    teleop_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()