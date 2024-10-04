#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray
import math

import numpy as np
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.01, self.update_odometry)

        self.wheel_radius = 0.105  # Радиус мотор-колеса (в метрах)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0  # Скорость мотор-колеса
        self.steering_angle = 0.0  # Угол поворота

        # Подписка на скорости и угол поворота
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        # print(msg.velocity[msg.name.index('drive_wheel_joint')])
        # print(msg.position[msg.name.index('steering_joint')])
        self.v = msg.velocity[msg.name.index('drive_wheel_joint')]
        self.steering_angle = msg.position[msg.name.index('steering_joint')]
        # print(f"v = {self.v} thet= {self.steering_angle}")



    # def drive_callback(self, msg):
    #     self.v = msg.data[0]  # Скорость мотор-колеса
    #
    # def steering_callback(self, msg):
    #     self.steering_angle = msg.data[0]  # Угол поворота

    def update_odometry(self):
        dt = 0.01  # Интервал времени обновления
        v_x = self.v * math.cos(self.steering_angle)*self.wheel_radius
        v_y = self.v * math.sin(self.steering_angle)*self.wheel_radius
        # print(f"v = {self.v} thet= {self.steering_angle}")
        # print(f"vx = {v_x} vy= {v_y}")


        self.x += v_x * dt
        self.y += v_y * dt
        self.theta += self.v * dt * math.tan(self.steering_angle) / self.wheel_radius
        print(self.x, self.y)


        # Публикация одометрии
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        euler = get_quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=euler[0], y=euler[1], z=euler[2], w=euler[3])

        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.angular.z = self.v / self.wheel_radius

        # print(odom)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    odometry_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





#
#
# import rclpy
# import threading
# from std_msgs.msg import Float64MultiArray
# import sys, select, termios, tty
#
#
# if sys.platform == 'win32':
#     import msvcrt
# else:
#     import termios
#     import tty
#
#
#
# # Define control keys and increments
# drive_wheel_forward = 'w'
# drive_wheel_backward = 's'
# stop_key = 'k'
# steering_right = 'd'
# steering_left = 'a'
# increase_speed = 'e'
# decrease_speed = 'q'
# increase_turn_rate = 'z'
# decrease_turn_rate = 'c'
# exit_key = 'x'
#
# # Speed and turn multipliers
# speed_multiplier = 1.0
# turn_rate_multiplier = 1.0
# speed_step = 0.1
# turn_rate_step = 0.1
#
# # Initial commands
# drive_command = 0.0
# steering_command = 0.0
#
# print("Control Your Robot!")
# print("---------------------------")
# print(f"Move wheel forward/backward: '{drive_wheel_forward}' / '{drive_wheel_backward}'")
# print(f"Turn steering left/right: '{steering_left}' / '{steering_right}'")
# print(f"Increase/decrease speed: '{increase_speed}' / '{decrease_speed}'")
# print(f"Increase/decrease turn rate: '{increase_turn_rate}' / '{decrease_turn_rate}'")
# print(f"Exit: '{exit_key}'")
#
#
# def getKey(settings):
#     if sys.platform == 'win32':
#         # getwch() returns a string on Windows
#         key = msvcrt.getwch()
#     else:
#         tty.setraw(sys.stdin.fileno())
#         # sys.stdin.read() returns a string on Linux
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key
#
#
# def saveTerminalSettings():
#     if sys.platform == 'win32':
#         return None
#     return termios.tcgetattr(sys.stdin)
#
#
# def restoreTerminalSettings(old_settings):
#     if sys.platform == 'win32':
#         return
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
#
#
# def vels(speed, turn):
#     return 'currently:\tspeed %s\tturn %s ' % (speed, turn)
#
#
# def main():
#     settings = saveTerminalSettings()
#     rclpy.init()
#
#     node = rclpy.create_node('teleop_node')
#     drive_publisher = node.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)
#     steering_publisher = node.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
#
#     spinner = threading.Thread(target=rclpy.spin, args=(node,))
#     spinner.start()
#
#     drive_msg = Float64MultiArray()
#     steering_msg = Float64MultiArray()
#
#     try:
#         while rclpy.ok():
#             key = getKey(settings)
#             if key == drive_wheel_forward:
#                 drive_command = 1.0 * speed_multiplier
#             elif key == drive_wheel_backward:
#                 drive_command = -1.0 * speed_multiplier
#             elif key == steering_left:
#                 steering_command = 1.0 * turn_rate_multiplier
#             elif key == steering_right:
#                 steering_command = -1.0 * turn_rate_multiplier
#             elif key == increase_speed:
#                 speed_multiplier += speed_step
#                 print(f"Speed multiplier: {speed_multiplier}")
#             elif key == decrease_speed:
#                 speed_multiplier = max(0.0, speed_multiplier - speed_step)
#                 print(f"Speed multiplier: {speed_multiplier}")
#             elif key == increase_turn_rate:
#                 turn_rate_multiplier += turn_rate_step
#                 print(f"Turn rate multiplier: {turn_rate_multiplier}")
#             elif key == decrease_turn_rate:
#                 turn_rate_multiplier = max(0.0, turn_rate_multiplier - turn_rate_step)
#                 print(f"Turn rate multiplier: {turn_rate_multiplier}")
#             else:
#                 steering_command = 0
#                 drive_command = 0
#                 if key == stop_key:
#                     break
#
#             # Publishing the drive command
#             drive_msg.data = [drive_command]
#             drive_publisher.publish(drive_msg)
#
#             # Publishing the steering command
#             steering_msg.data = [steering_command]
#             steering_publisher.publish(steering_msg)
#     except Exception as e:
#         print(e)
#
#     finally:
#         steering_command = 0
#         drive_command = 0
#
#         drive_msg.data = [drive_command]
#         drive_publisher.publish(drive_msg)
#
#         steering_msg.data = [steering_command]
#         steering_publisher.publish(steering_msg)
#
#         rclpy.shutdown()
#         spinner.join()
#
#         restoreTerminalSettings(settings)
#
#
#
# if __name__ == '__main__':
#     main()
#











# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import sys, select, termios, tty
#
# msg = """
# Control Your Robot!
# ---------------------------
# Moving around:
#    w
# a     d
#    s
#
# w/s : Drive forward/backward
# a/d : Turn left/right
#
# q/e : Increase/decrease driving speed by 10%
# z/c : Increase/decrease steering speed by 10%
#
# CTRL-C to quit
# """
#
# moveBindings = {
#     'w': (1, 0),
#     's': (-1, 0),
#     'a': (0, 1),
#     'd': (0, -1),
# }
#
# speedBindings = {
#     'q': (1.1, 1.0),
#     'e': (0.9, 1.0),
#     'z': (1.0, 1.1),
#     'c': (1.0, 0.9),
# }
#
#
# class TeleopNode(Node):
#     def __init__(self):
#         super().__init__('teleop_node')
#         self.drive_wheel_pub = self.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)
#         self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
#
#         self.speed = 1.0
#         self.turn_speed = 1.0
#         self.drive = 0.0
#         self.turn = 0.0
#
#         self.settings = termios.tcgetattr(sys.stdin)
#         self.timer = self.create_timer(0.1, self.update_movement)
#         print(msg)
#
#     def getKey(self):
#         tty.setraw(sys.stdin.fileno())
#         select.select([sys.stdin], [], [], 0)
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
#         return key
#
#     def update_movement(self):
#         drive_msg = Float64MultiArray()
#         steering_msg = Float64MultiArray()
#
#         drive_msg.data = [self.drive * self.speed]
#         steering_msg.data = [self.turn * self.turn_speed]
#
#         self.drive_wheel_pub.publish(drive_msg)
#         self.steering_pub.publish(steering_msg)
#
#     def run(self):
#         try:
#             while True:
#                 key = self.getKey()
#                 if key in moveBindings.keys():
#                     self.drive = moveBindings[key][0]
#                     self.turn = moveBindings[key][1]
#                 elif key in speedBindings.keys():
#                     self.speed *= speedBindings[key][0]
#                     self.turn_speed *= speedBindings[key][1]
#                     print(f'Speed: {self.speed}, Turn Speed: {self.turn_speed}')
#                 elif key == '\x03':  # CTRL-C
#                     break
#                 else:
#                     self.drive = 0.0
#                     self.turn = 0.0
#
#         except Exception as e:
#             print(e)
#
#         finally:
#             self.update_movement()
#             self.drive = 0.0
#             self.turn = 0.0
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = TeleopNode()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()





#
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import sys, select, termios, tty
#
# msg = """
# Control Your Robot!
# ---------------------------
# Moving around:
#    w
# a     d
#    s
#
# w/s : Drive forward/backward
# a/d : Turn left/right
#
# q/e : Increase/decrease driving speed by 10%
# z/c : Increase/decrease steering speed by 10%
#
# CTRL-C to quit
# """
#
# moveBindings = {
#     'w': (1, 0),
#     's': (-1, 0),
#     'a': (0, 1),
#     'd': (0, -1),
# }
#
# speedBindings = {
#     'q': (1.1, 1.0),
#     'e': (0.9, 1.0),
#     'z': (1.0, 1.1),
#     'c': (1.0, 0.9),
# }
#
#
# class TeleopNode(Node):
#     def __init__(self):
#         super().__init__('teleop_node')
#         self.drive_wheel_pub = self.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)
#         self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
#
#         self.speed = 1.0
#         self.turn_speed = 1.0
#         self.drive = 0.0
#         self.turn = 0.0
#
#         self.settings = termios.tcgetattr(sys.stdin)
#         self.timer = self.create_timer(0.1, self.update_movement)
#         print(msg)
#
#     def getKey(self):
#         tty.setraw(sys.stdin.fileno())
#         select.select([sys.stdin], [], [], 0)
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
#         return key
#
#     def update_movement(self):
#         drive_msg = Float64MultiArray()
#         steering_msg = Float64MultiArray()
#
#         drive_msg.data = [self.drive * self.speed]
#         steering_msg.data = [self.turn * self.turn_speed]
#
#         self.drive_wheel_pub.publish(drive_msg)
#         self.steering_pub.publish(steering_msg)
#
#     def run(self):
#         try:
#             while True:
#                 key = self.getKey()
#                 if key in moveBindings.keys():
#                     self.drive = moveBindings[key][0]
#                     self.turn = moveBindings[key][1]
#                     self.update_movement()
#                     self.drive = 0.0
#                     self.turn = 0.0
#                 elif key in speedBindings.keys():
#                     self.speed *= speedBindings[key][0]
#                     self.turn_speed *= speedBindings[key][1]
#                     print(f'Speed: {self.speed}, Turn Speed: {self.turn_speed}')
#                 elif key == '\x03':  # CTRL-C
#                     break
#                 else:
#                     self.drive = 0.0
#                     self.turn = 0.0
#
#         except Exception as e:
#             print(e)
#
#         finally:
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = TeleopNode()
#     node.run()
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
