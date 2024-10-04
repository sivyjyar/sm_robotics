#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal_node')

        # Подписываемся на одометрию
        self.odom_subscriber = self.create_subscription(
            Odometry, '/gazebo_odom', self.odom_callback, 10)

        # Публикация в топики для управления мотором и рулем
        self.drive_wheel_publisher = self.create_publisher(
            Float64MultiArray, '/drive_wheel_controller/commands', 10)
        self.steering_publisher = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # Задаем целевое положение (x, y, theta)
        self.goal_x = 0.0
        self.goal_y = -2.0
        self.goal_theta = 0

        # Текущие данные о положении
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Периодическое выполнение функции движения
        self.timer = self.create_timer(0.1, self.move_to_goal)

    def odom_callback(self, msg):
        # Извлекаем текущие координаты робота
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Извлекаем угол yaw (theta) из кватерниона
        orientation_q = msg.pose.pose.orientation
        (_, _, self.current_theta) = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        # Преобразуем кватернион в углы Эйлера
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  # только yaw (theta) интересен для движения по плоскости

    def move_to_goal(self):
        # Вычисляем ошибку по координатам
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y

        # Вычисляем расстояние до цели
        distance_to_goal = math.sqrt(error_x ** 2 + error_y ** 2)

        # Если расстояние меньше порога, останавливаем робота
        if distance_to_goal < 0.1:
            self.stop_robot()
            return

        # Вычисляем желаемый угол движения
        goal_angle = math.atan2(error_y, error_x)

        # Вычисляем ошибку по углу
        angle_error = goal_angle - self.current_theta

        # Ограничиваем угол поворота до диапазона от -pi до pi
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # Управляем углом поворота колеса
        self.control_steering(angle_error)

        # Управляем скоростью
        self.control_speed(distance_to_goal, angle_error)

    def control_steering(self, angle_error):
        # Простой пропорциональный контроллер для угла поворота
        steering_command = Float64MultiArray()
        steering_angle = max(-0.5, min(0.5, angle_error))  # ограничиваем угол
        steering_command.data = [steering_angle]
        self.steering_publisher.publish(steering_command)

    def control_speed(self, distance, angle_error):
        # Простой пропорциональный контроллер для скорости
        drive_command = Float64MultiArray()

        # Если ошибка угла велика, уменьшаем скорость
        if abs(angle_error) > 0.1:
            speed = 1  # маленькая скорость для поворота
        else:
            speed = 5.0 * min(1.0, distance)  # нормальная скорость

        drive_command.data = [speed]
        self.drive_wheel_publisher.publish(drive_command)

    def stop_robot(self):
        # Останавливаем робота, отправляя 0 в топики
        stop_drive_command = Float64MultiArray()
        stop_drive_command.data = [0.0]
        self.drive_wheel_publisher.publish(stop_drive_command)

        stop_steering_command = Float64MultiArray()
        stop_steering_command.data = [0.0]
        self.steering_publisher.publish(stop_steering_command)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
