#!/usr/bin/env python3

import math
import rclpy
from enum import Enum, auto
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

def yaw_from_quaternion(q):
    """Converte um quaternion (geometry_msgs/Quaternion) para o ângulo yaw."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    """Normaliza um ângulo para o intervalo (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class State(Enum):
    TURNING = auto()    # alinhando o yaw com o alvo, parado
    MOVING = auto()      # avançando em linha reta na direção do alvo
    AVOIDING = auto()    # obstáculo próximo detectado, desviando

class NavigatorNode(Node):

    # --- Parâmetros de navegação -------------------------------------------------
    GOAL_TOLERANCE = 0.15          # [m] distância para considerar o alvo alcançado
    YAW_TOLERANCE = 0.08           # [rad] (~4.5 graus) considera o robô "alinhado"

    SAFE_DISTANCE = 0.40           # [m] distância frontal que dispara o desvio
    CLEAR_DISTANCE = 0.55          # [m] distância para considerar o caminho livre de novo
    FRONT_CONE_DEG = 35            # [graus] meia-abertura do cone frontal analisado
    SIDE_CONE_DEG = 90             # [graus] meia-abertura usada para escolher o lado livre

    MAX_LINEAR_VEL = 0.20          # [m/s] (limite do TurtleBot3 Burger é 0.22 m/s)
    MAX_ANGULAR_VEL = 1.5          # [rad/s]
    AVOID_ANGULAR_VEL = 0.9        # [rad/s] velocidade de giro durante o desvio
    K_ANGULAR = 2.0                # ganho proporcional do controle angular em MOVING

    def __init__(self):
        super().__init__('navigator_node')

        self.home = (-2.0, 2.0)  # posição inicial, conforme o enunciado
        self.targets = [
            ('Verde', 2.20, 2.20),
            ('Vermelho', 2.15, -2.15),
            ('Azul', -2.16, -2.16),
            ('Laranja', -2.00, 1.20),
        ]

        self.current_target_index = 0
        self.going_home = False
        self.mission_done = False
        self.state = State.TURNING
        self.avoid_direction = 1.0  # +1 = gira para a esquerda, -1 = direita

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.latest_scan = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Loop de controle a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Navigator node iniciado (poses gravadas + desvio reativo). '
            'Aguardando odometria e scan...')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def current_goal(self):
        if self.going_home:
            return self.home
        _, gx, gy = self.targets[self.current_target_index]
        return (gx, gy)

    def sector_min_distance(self, center_deg, half_width_deg):
        """Menor distância do /scan dentro de um setor angular (graus, no
        referencial do robô; 0 = frente, positivo = esquerda)."""
        scan = self.latest_scan
        if scan is None:
            return float('inf')

        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)
        lo, hi = center - half_width, center + half_width

        angle = scan.angle_min
        min_d = float('inf')
        for r in scan.ranges:
            a = normalize_angle(angle)
            if lo <= a <= hi and math.isfinite(r) and r > scan.range_min:
                min_d = min(min_d, r)
            angle += scan.angle_increment
        return min_d

    def front_distance(self):
        return self.sector_min_distance(0.0, self.FRONT_CONE_DEG)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.odom_received or self.mission_done:
            return

        gx, gy = self.current_goal()
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)

        if distance < self.GOAL_TOLERANCE:
            self.stop_robot()
            self.on_goal_reached()
            return

        front_dist = self.front_distance()

        if self.state != State.AVOIDING and front_dist < self.SAFE_DISTANCE:
            self.state = State.AVOIDING
            # Escolhe o lado com mais espaço livre para girar
            left = self.sector_min_distance(90.0, self.SIDE_CONE_DEG / 2)
            right = self.sector_min_distance(-90.0, self.SIDE_CONE_DEG / 2)
            self.avoid_direction = 1.0 if left >= right else -1.0
            self.get_logger().info(
                f'Obstáculo a {front_dist:.2f} m à frente. Desviando para '
                f'{"esquerda" if self.avoid_direction > 0 else "direita"}.')

        if self.state == State.AVOIDING:
            if front_dist > self.CLEAR_DISTANCE:
                # Caminho livre de novo: volta a se orientar para o alvo
                self.state = State.TURNING
            else:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = self.avoid_direction * self.AVOID_ANGULAR_VEL
                self.cmd_pub.publish(cmd)
                return

        # --- Controle "gira e anda" em direção à pose gravada ---
        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - self.yaw)

        if abs(yaw_error) > self.YAW_TOLERANCE:
            self.state = State.TURNING
        else:
            self.state = State.MOVING

        cmd = Twist()
        if self.state == State.TURNING:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self.MAX_ANGULAR_VEL,
                min(self.MAX_ANGULAR_VEL, self.K_ANGULAR * yaw_error),
            )
        else:  # MOVING
            cmd.linear.x = self.MAX_LINEAR_VEL
            # Pequena correção proporcional para manter o rumo durante o avanço
            cmd.angular.z = max(
                -self.MAX_ANGULAR_VEL,
                min(self.MAX_ANGULAR_VEL, self.K_ANGULAR * yaw_error),
            )

        self.cmd_pub.publish(cmd)

    def on_goal_reached(self):
        self.state = State.TURNING
        if self.going_home:
            self.get_logger().info('Retornou à posição inicial.')
            self.going_home = False
            self.current_target_index += 1
            if self.current_target_index >= len(self.targets):
                self.mission_done = True
                self.get_logger().info('Missão concluída: todos os alvos visitados.')
        else:
            name, gx, gy = self.targets[self.current_target_index]
            self.get_logger().info(
                f'Alvo {name} alcançado ({gx}, {gy}). Retornando à posição inicial.')
            self.going_home = True

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
