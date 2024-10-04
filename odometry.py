#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
import math
import numpy as np


def normalize2pi(angle):
    if angle == math.pi:
        return angle
    angle = (angle+math.pi)/(2*math.pi)
    if angle < 0:
        angle = angle + math.pi
    return angle - math.pi


class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.update_odometry)

        self.wheel_radius = 0.105  # Радиус мотор-колеса (в метрах)
        self.wheelbase = 1.285  # Расстояние от мотор-колеса до доп. колес

        # Начальные значения
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

        # Для проверки одометрии используются данные о положении робота в Gazebo:
        self.gazebo_subscription = self.create_subscription(
            Odometry,
            '/gazebo_odom',
            self.gazebo_callback,
            10)

        self.odom_broadcaster = TransformBroadcaster(self)

    # Запрос положения робота в Gazebo:
    def gazebo_callback(self, msg):
        self.gazebo_x = msg.pose.pose.position.x
        self.gazebo_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        self.gazebo_theta = yaw

    # Получение данных с "энкодеров" в Gazebo:
    def joint_state_callback(self, msg):
        self.v = round(msg.velocity[msg.name.index('drive_wheel_joint')], 4)
        # Получаем данные с датчика, нормируем от 0 до 2Пи, округляем до 3 знака
        # self.steering_angle = round(msg.position[msg.name.index('steering_joint')], 3)

        self.steering_angle = msg.position[msg.name.index('steering_joint')] % (2 * math.pi)
        self.steering_angle = round(self.steering_angle, 3)
        # print(f"v={self.v}, a={self.steering_angle}")


    # Расчет одометрии:
    def update_odometry(self):
        dt = 0.1  # Интервал времени обновления
        # d = self.v * self.wheel_radius * dt #Расстояние пройденное за dt
        self.w = self.v * self.wheel_radius

        if self.steering_angle != 0:

            # turning_radius = self.wheelbase / math.tan(self.steering_angle)  # Радиус поворота
            # turn_angle = d / turning_radius
            R_sw = self.wheelbase / math.sin(self.steering_angle)

            self.theta += self.w * dt / R_sw
            self.x += self.w * dt * math.cos(self.steering_angle+self.theta)
            self.y += self.w * dt * math.sin(self.steering_angle+self.theta)

            # self.theta += self.w * dt / R_sw
            # self.x += self.w * dt * math.sin(-self.theta)
            # self.y -= self.w * dt * math.cos(self.theta)

            # angular_velocity = self.v / R_sw
            # x_c = self.x - R_sw * math.sin(self.theta)
            # y_c = self.y + R_sw * math.cos(self.theta)
            #
            # # Расчет новых координат
            # self.x = x_c + R_sw * math.sin(self.theta + turn_angle)
            # self.y = y_c - R_sw * math.cos(self.theta + turn_angle)
            # self.theta = (self.theta+turn_angle) % (2 * math.pi)

            # print('ANGULAR')
            print(self.w)
            print(self.x, self.y, self.theta)
            print(self.gazebo_x, self.gazebo_y, self.gazebo_theta)

        else:
            # Прямолинейное движение
            self.x += self.w * dt * math.cos(self.theta)
            self.y += self.w * dt * math.sin(self.theta)
            self.theta = self.theta % (2 * math.pi)
            # self.theta = abs(self.theta/2/math.pi)

            # print(self.x, self.y, self.theta)

        # print(self.x, self.y, self.theta)
        print("\n")




        # if self.steering_angle != 0:
        #     turning_radius = self.wheelbase / math.tan(self.steering_angle)  # Радиус поворота
        #     angular_velocity = self.v / turning_radius  # Угловая скорость
        #
        #         # Расчет новых координат
        #     self.x += self.v * math.cos(self.theta) * dt
        #     self.y += self.v * math.sin(self.theta) * dt
        #     self.theta += angular_velocity * dt
        # else:
        #         # Прямолинейное движение
        #     self.x += self.v * math.cos(self.theta) * dt
        #     self.y += self.v * math.sin(self.theta) * dt

            # Нормализация угла theta в диапазон [-pi, pi]
        # self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
        #
        # print(self.x, self.y)


        # # Публикация одометрии
        # odom = Odometry()
        # odom.header.frame_id = 'odom'
        # odom.child_frame_id = 'base_link'
        # odom.header.stamp = self.get_clock().now().to_msg()
        #
        # odom.pose.pose.position.x = self.x
        # odom.pose.pose.position.y = self.y
        # quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        # odom.pose.pose.orientation = Quaternion(quat)
        #
        # odom.twist.twist.linear.x = 0
        # odom.twist.twist.linear.y = 0
        # # odom.twist.twist.linear.x = v_x
        # # odom.twist.twist.linear.y = v_y
        # odom.twist.twist.angular.z = self.v / self.wheel_radius
        #
        # # print(odom)
        # self.odom_pub.publish(odom)

        # # Публикация трансформации
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation = Quaternion(*quat)
        #
        # self.odom_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    odometry_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
