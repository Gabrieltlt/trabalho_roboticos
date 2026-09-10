#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)

        start = (-2.0, 2.0)

        target_paths = [
            ('verde', [
                (0.0, 1.8),
                (0.40, 0.90),
                (1.75, 0.98),
                (2.0, 2.0),
            ]),
            ('vermelho', [
                (0.0, 1.8),
                (0.0, -0.5),
                (1.5, -0.0),
                (2.10, -1.89),
            ]),
            ('azul', [
                (-0.0, 2.0),
                (0.0, -2.0),
                (-1.9, -2.0),
            ]),
            ('laranja', [
                (-0.4, 2.0),
                (-0.4, 0.0),
                (-1.5, 0.0),
                (-1.8, 1.1),
            ]),
        ]

        self.goals = []
        self.goal_names = []
        for name, mid_points in target_paths:
            for point in mid_points[:-1]:
                self.goals.append(point)
                self.goal_names.append(f'rumo ao {name}')
            self.goals.append(mid_points[-1])
            self.goal_names.append(f'alvo {name}')

            for point in list(reversed(mid_points[:-1])):
                self.goals.append(point)
                self.goal_names.append('retorno')
            self.goals.append(start)
            self.goal_names.append('retorno')

        self.current_goal = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan = None

        self.goal_tolerance = 0.20
        self.slow_radius = 0.8

        self.max_linear = 0.18
        self.max_angular = 1.0

        self.waiting = False
        self.wait_until = 0.0

        self.wait_points = {i for i, n in enumerate(self.goal_names) if n.startswith('alvo')}

        self.finished = False

        self.get_logger().info('Navegador iniciado!')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        self.scan = msg

    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def obstacle_emergency(self):
        """Checagem simples e sem estado: reage à leitura atual do laser,
        sem precisar 'lembrar' que estava desviando entre ciclos."""
        if self.scan is None:
            return False, 0.0

        ranges = self.scan.ranges
        center = len(ranges) // 2

        front_sector = ranges[max(0, center - 15):min(len(ranges), center + 15)]
        valid = [r for r in front_sector if not math.isinf(r) and not math.isnan(r)]

        if not valid:
            return False, 0.0

        front = min(valid)

        if front < 0.35:
            half = len(front_sector) // 2
            left_vals = [r for r in front_sector[half:] if not math.isinf(r) and not math.isnan(r)]
            right_vals = [r for r in front_sector[:half] if not math.isinf(r) and not math.isnan(r)]
            left_clear = min(left_vals) if left_vals else 999.0
            right_clear = min(right_vals) if right_vals else 999.0
            turn_dir = 1.0 if left_clear >= right_clear else -1.0
            return True, turn_dir * 0.8

        return False, 0.0

    def control_loop(self):
        if self.finished:
            return

        if self.current_goal >= len(self.goals):
            self.finished = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Missão concluída!')
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.waiting:
            now = self.get_clock().now().nanoseconds / 1e9
            self.cmd_pub.publish(Twist())
            if now >= self.wait_until:
                self.waiting = False
                self.current_goal += 1
            return

        goal_x, goal_y = self.goals[self.current_goal]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            if self.current_goal in self.wait_points:
                self.get_logger().info(f'Chegou: {self.goal_names[self.current_goal]} '
                                        f'({goal_x:.2f}, {goal_y:.2f})')
                self.waiting = True
                self.wait_until = (self.get_clock().now().nanoseconds / 1e9 + 2.0)
                self.cmd_pub.publish(Twist())
            else:
                self.current_goal += 1
            return

        desired_angle = math.atan2(dy, dx)
        angle_error = self.normalize(desired_angle - self.yaw)

        angular = 1.0 * angle_error
        angular = max(-self.max_angular, min(self.max_angular, angular))

        if abs(angle_error) > 1.0:
            linear = 0.05
        elif abs(angle_error) > 0.5:
            linear = 0.10
        else:
            linear = self.max_linear

        if distance < self.slow_radius:
            linear *= 0.5

        blocked, turn = self.obstacle_emergency()

        cmd = Twist()
        if blocked:
            cmd.linear.x = 0.0
            cmd.angular.z = turn
        else:
            cmd.linear.x = linear
            cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()