#! /usr/bin/env python3
import rclpy
import threading
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
---------------------------
Moving around:
   w
a     d
   s

w/s : Drive forward/backward
a/d : Turn left/right

e/q : Increase/decrease driving speed by 10%
c/z : Increase/decrease steering speed by 10%

"x" to quit
---------------------------
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    # Define control keys and increments
    drive_wheel_forward = 'w'
    drive_wheel_backward = 's'
    steering_right = 'd'
    steering_left = 'a'
    increase_speed = 'e'
    decrease_speed = 'q'
    increase_turn_rate = 'z'
    decrease_turn_rate = 'c'
    exit_key = 'x'

    # Speed and turn multipliers
    speed_multiplier = 1.0
    turn_rate_multiplier = 0.05
    speed_step = 0.1
    turn_rate_step = 0.05

    # Initial commands
    drive_command = 0.0
    steering_command = 0.0


    print(msg)

    settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('teleop_node')
    drive_publisher = node.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)
    steering_publisher = node.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    drive_msg = Float64MultiArray()
    steering_msg = Float64MultiArray()

    try:
        while rclpy.ok():
            key = getKey(settings)
            if key == drive_wheel_forward:
                drive_command = 1.0 * speed_multiplier
            elif key == drive_wheel_backward:
                drive_command = -1.0 * speed_multiplier
            elif key == steering_left:
                steering_command += 1.0 * turn_rate_multiplier
            elif key == steering_right:
                steering_command += -1.0 * turn_rate_multiplier
            elif key == increase_speed:
                speed_multiplier += speed_step
                print(f"Speed multiplier: {speed_multiplier}")
            elif key == decrease_speed:
                speed_multiplier = max(speed_step, speed_multiplier - speed_step)
                print(f"Speed multiplier: {speed_multiplier}")
            elif key == increase_turn_rate:
                turn_rate_multiplier += turn_rate_step
                print(f"Turn rate multiplier: {turn_rate_multiplier}")
            elif key == decrease_turn_rate:
                turn_rate_multiplier = max(turn_rate_step, turn_rate_multiplier - turn_rate_step)
                print(f"Turn rate multiplier: {turn_rate_multiplier}")
            else:
                drive_command = 0
                if key == exit_key:
                    break

            # Publishing the drive command
            drive_msg.data = [drive_command]
            drive_publisher.publish(drive_msg)

            # Publishing the steering command
            steering_msg.data = [steering_command]
            steering_publisher.publish(steering_msg)
    except Exception as e:
        print(e)

    finally:
        drive_command = 0

        drive_msg.data = [drive_command]
        drive_publisher.publish(drive_msg)

        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)



if __name__ == '__main__':
    main()






























#

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import sys, select, termios, tty
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
#
# class TeleopNode(Node):
#     def __init__(self):
#         super().__init__('teleop_node')
#         self.drive_publisher = self.create_publisher(Float64MultiArray, '/drive_wheel_controller/commands', 10)
#         self.steering_publisher = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
#
#         print("Control Your Robot!")
#         print("---------------------------")
#         print(f"Move wheel forward/backward: '{drive_wheel_forward}' / '{drive_wheel_backward}'")
#         print(f"Turn steering left/right: '{steering_left}' / '{steering_right}'")
#         print(f"Increase/decrease speed: '{increase_speed}' / '{decrease_speed}'")
#         print(f"Increase/decrease turn rate: '{increase_turn_rate}' / '{decrease_turn_rate}'")
#         print(f"Exit: '{exit_key}'")
#
#         self.run()
#
#     def get_key(self):
#         tty.setraw(sys.stdin.fileno())
#         select.select([sys.stdin], [], [], 0)
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
#         return key
#
#     def run(self):
#         global drive_command, steering_command, speed_multiplier, turn_rate_multiplier
#
#         try:
#             while rclpy.ok():
#                 key = self.get_key()
#
#                 if key == drive_wheel_forward:
#                     drive_command = 1.0 * speed_multiplier
#                 elif key == drive_wheel_backward:
#                     drive_command = -1.0 * speed_multiplier
#                 elif key == steering_left:
#                     steering_command = 1.0 * turn_rate_multiplier
#                 elif key == steering_right:
#                     steering_command = -1.0 * turn_rate_multiplier
#                 elif key == increase_speed:
#                     speed_multiplier += speed_step
#                     print(f"Speed multiplier: {speed_multiplier}")
#                 elif key == decrease_speed:
#                     speed_multiplier = max(0.0, speed_multiplier - speed_step)
#                     print(f"Speed multiplier: {speed_multiplier}")
#                 elif key == increase_turn_rate:
#                     turn_rate_multiplier += turn_rate_step
#                     print(f"Turn rate multiplier: {turn_rate_multiplier}")
#                 elif key == decrease_turn_rate:
#                     turn_rate_multiplier = max(0.0, turn_rate_multiplier - turn_rate_step)
#                     print(f"Turn rate multiplier: {turn_rate_multiplier}")
#                 elif key == stop_key:
#                     steering_command = 0
#                     drive_command = 0
#                 elif key == exit_key:
#                     break
#
#                 # Publishing the drive command
#                 drive_msg = Float64MultiArray()
#                 drive_msg.data = [drive_command]
#                 self.drive_publisher.publish(drive_msg)
#
#                 # Publishing the steering command
#                 steering_msg = Float64MultiArray()
#                 steering_msg.data = [steering_command]
#                 self.steering_publisher.publish(steering_msg)
#
#         except Exception as e:
#             print(e)
#         finally:
#             # Stop the robot when the script ends
#             drive_msg = Float64MultiArray()
#             drive_msg.data = [0.0]
#             self.drive_publisher.publish(drive_msg)
#
#             steering_msg = Float64MultiArray()
#             steering_msg.data = [0.0]
#             self.steering_publisher.publish(steering_msg)
#
#             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
#             print("Shutting down.")
#
#
#             rclpy.shutdown()
#
#
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = TeleopNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     # rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()
#
#
#
#
#
#
#
#
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
