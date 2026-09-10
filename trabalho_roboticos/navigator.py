#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class State(Enum):
    SEEKING = 0          # perseguindo o alvo (gira + avança de forma combinada)
    AVOIDING = 1          # desviando de obstáculo
    ARRIVED_PAUSE = 2     # parado no alvo por um tempo fixo
    LEAVING = 3           # recuando um pouco antes de seguir pro próximo destino
    MISSION_DONE = 4


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator_node')

        # -------- Parâmetros da missão (Tabela 2 do enunciado) --------
        self.start_pose = (-2.0, 2.0)
        self.targets = [
            ('verde', 2.20, 2.20),
            ('vermelho', 2.15, -2.15),
            ('azul', -2.16, -2.16),
            ('laranja', -2.00, 1.20),
        ]

        # Controle de perseguição de alvo
        self.goal_tolerance = 0.18        # m
        self.turn_in_place_angle = 0.5    # rad (~28°) - acima disso, só gira, não avança
        self.max_linear_speed = 0.18      # m/s
        self.max_angular_speed = 1.0      # rad/s
        self.kp_angular = 1.6
        self.kp_linear = 0.6

        # Desvio de obstáculos
        self.safe_distance = 0.45         # m - distância mínima frontal
        self.front_angle_window = 45      # graus para cada lado da frente
        self.avoid_exit_ticks_min = 12    # ciclos mínimos em AVOIDING antes de poder sair (~1.2s)
        self.avoid_creep_after_ticks = 8  # a partir de quando libera avanço leve durante o desvio

        # Pausa ao chegar no alvo / recuo antes de seguir
        self.arrived_pause_ticks = 15     # ~1.5s parado no alvo
        self.leaving_ticks = 12           # ~1.2s recuando
        self.leaving_speed = -0.06        # m/s (recuo)

        # Detecção de robô travado (mínimo local / oscilação)
        self.stuck_move_threshold = 0.05   # m
        self.stuck_yaw_threshold = 0.15    # rad (~8.5°)
        self.stuck_ticks_threshold = 50    # ~5s sem progresso -> travado
        self.recovery_ticks = 15           # duração da manobra de recuperação

        # -------- Estado interno --------
        self.state = State.SEEKING
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        self.avoid_direction = 1.0   # +1 = esquerda, -1 = direita (definido ao entrar em AVOIDING)
        self.avoid_ticks = 0
        self.pause_ticks = 0
        self.leave_ticks = 0
        self.recovery_ticks_left = 0

        self.stuck_ticks = 0
        self.stuck_ref_x = None
        self.stuck_ref_y = None
        self.stuck_ref_yaw = None

        # Fila de "sub-metas": para cada alvo -> ir até o alvo, depois voltar ao início
        self.mission_queue = []
        for name, tx, ty in self.targets:
            self.mission_queue.append((f'alvo {name}', tx, ty))
            self.mission_queue.append(('retorno', self.start_pose[0], self.start_pose[1]))
        self.mission_index = 0
        self.current_label, self.goal_x, self.goal_y = self.mission_queue[0]

        # -------- ROS I/O --------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Navigator iniciado. Aguardando odometria e laser...')
        self.get_logger().info(f'Próximo destino: {self.current_label} '
                                f'({self.goal_x:.2f}, {self.goal_y:.2f})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    # ------------------------------------------------------------------
    # Utilidades de leitura do laser
    # ------------------------------------------------------------------
    def _sector_min(self, center_deg, half_width_deg):
        """Retorna a menor distância válida num setor angular do /scan."""
        if not self.scan_ranges:
            return float('inf')

        center_rad = math.radians(center_deg)
        half_width_rad = math.radians(half_width_deg)

        min_range = float('inf')
        for i, r in enumerate(self.scan_ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.01:
                continue
            angle = self.scan_angle_min + i * self.scan_angle_increment
            diff = math.atan2(math.sin(angle - center_rad), math.cos(angle - center_rad))
            if abs(diff) <= half_width_rad:
                min_range = min(min_range, r)
        return min_range

    def front_distance(self):
        return self._sector_min(0, self.front_angle_window)

    def left_distance(self):
        return self._sector_min(90, 45)

    def right_distance(self):
        return self._sector_min(-90, 45)

    def rear_distance(self):
        return self._sector_min(180, 45)

    # ------------------------------------------------------------------
    # Loop de controle
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.odom_received or not self.scan_ranges:
            return

        if self.state == State.MISSION_DONE:
            self.stop_robot()
            return

        # -------- Manobra de recuperação tem prioridade sobre tudo --------
        if self.recovery_ticks_left > 0:
            self.recovery_ticks_left -= 1
            cmd = Twist()
            cmd.linear.x = -0.08
            cmd.angular.z = self.max_angular_speed
            self.cmd_pub.publish(cmd)
            if self.recovery_ticks_left == 0:
                self._reset_stuck_reference()
            return

        # -------- Detecção de robô travado --------
        # só verifica durante SEEKING/AVOIDING, que são os estados onde
        # progresso é esperado; ARRIVED_PAUSE/LEAVING ficam parados de propósito
        if self.state in (State.SEEKING, State.AVOIDING):
            if self.stuck_ref_x is None:
                self._reset_stuck_reference()

            moved = math.hypot(self.x - self.stuck_ref_x, self.y - self.stuck_ref_y)
            yaw_diff = abs(math.atan2(math.sin(self.yaw - self.stuck_ref_yaw),
                                       math.cos(self.yaw - self.stuck_ref_yaw)))

            if moved < self.stuck_move_threshold and yaw_diff < self.stuck_yaw_threshold:
                self.stuck_ticks += 1
            else:
                self.stuck_ticks = 0
                self._reset_stuck_reference()

            if self.stuck_ticks > self.stuck_ticks_threshold:
                self.get_logger().warn('Robô travado (sem progresso). Executando recuperação.')
                self.stuck_ticks = 0
                self.recovery_ticks_left = self.recovery_ticks
                return
        else:
            self._reset_stuck_reference()
            self.stuck_ticks = 0

        front = self.front_distance()

        # -------- Log de diagnóstico periódico (a cada ~1s) --------
        if not hasattr(self, '_debug_tick'):
            self._debug_tick = 0
        self._debug_tick += 1
        if self._debug_tick % 10 == 0:
            dist_dbg = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
            angle_dbg = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            err_dbg = math.atan2(math.sin(angle_dbg - self.yaw), math.cos(angle_dbg - self.yaw))
            self.get_logger().info(
                f'[DEBUG] estado={self.state.name} pos=({self.x:.2f},{self.y:.2f}) '
                f'yaw={math.degrees(self.yaw):.1f}° alvo=({self.goal_x:.2f},{self.goal_y:.2f}) '
                f'dist={dist_dbg:.2f}m erro_ang={math.degrees(err_dbg):.1f}° front={front:.2f}m '
                f'esq={self.left_distance():.2f}m dir={self.right_distance():.2f}m')

        # -------- Máquina de estados principal --------
        if self.state == State.SEEKING:
            dist_to_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

            if dist_to_goal < self.goal_tolerance:
                self.state = State.ARRIVED_PAUSE
                self.pause_ticks = 0
                self.get_logger().info(f'Chegou em: {self.current_label} '
                                        f'({self.goal_x:.2f}, {self.goal_y:.2f})')
                self.stop_robot()
                return

            if front < self.safe_distance:
                self.state = State.AVOIDING
                self.avoid_ticks = 0
                # decide o lado do desvio uma vez só, e mantém enquanto durar o desvio
                self.avoid_direction = 1.0 if self.left_distance() > self.right_distance() else -1.0
                return

            self._seek_goal(dist_to_goal)

        elif self.state == State.AVOIDING:
            self.avoid_ticks += 1
            self._avoid_obstacle()

            # só volta a perseguir o alvo depois de um tempo mínimo girando
            # E com o caminho realmente livre — evita voltar cedo demais e
            # bater no mesmo obstáculo de novo (oscilação)
            if front >= self.safe_distance * 1.3 and self.avoid_ticks > self.avoid_exit_ticks_min:
                self.state = State.SEEKING

        elif self.state == State.ARRIVED_PAUSE:
            self.stop_robot()
            self.pause_ticks += 1
            if self.pause_ticks >= self.arrived_pause_ticks:
                self.state = State.LEAVING
                self.leave_ticks = 0

        elif self.state == State.LEAVING:
            self.leave_ticks += 1
            cmd = Twist()
            # só recua se não tiver nada logo atrás; senão só espera parado
            if self.rear_distance() > 0.25:
                cmd.linear.x = self.leaving_speed
            self.cmd_pub.publish(cmd)

            if self.leave_ticks >= self.leaving_ticks:
                self.stop_robot()
                self.advance_mission()

    # ------------------------------------------------------------------
    # Comportamentos
    # ------------------------------------------------------------------
    def _seek_goal(self, dist_to_goal):
        """Controle combinado: corrige o ângulo e avança ao mesmo tempo,
        reduzindo a velocidade linear quanto maior o erro angular."""
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_error = math.atan2(
            math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))

        cmd = Twist()
        cmd.angular.z = max(-self.max_angular_speed,
                             min(self.max_angular_speed, self.kp_angular * angle_error))

        if abs(angle_error) > self.turn_in_place_angle:
            cmd.linear.x = 0.0  # erro angular grande demais: só gira no lugar
        else:
            # reduz a velocidade linear proporcionalmente ao erro angular
            angular_factor = 1.0 - (abs(angle_error) / self.turn_in_place_angle)
            speed = min(self.max_linear_speed, self.kp_linear * dist_to_goal)
            cmd.linear.x = speed * max(0.0, angular_factor)

        self.cmd_pub.publish(cmd)

    def _avoid_obstacle(self):
        """Gira para o lado escolhido; libera avanço leve só depois de já
        ter girado por um tempo (evita roçar o obstáculo de raspão)."""
        cmd = Twist()
        cmd.linear.x = 0.06 if self.avoid_ticks > self.avoid_creep_after_ticks else 0.0
        cmd.angular.z = self.avoid_direction * self.max_angular_speed * 0.8
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _reset_stuck_reference(self):
        self.stuck_ref_x, self.stuck_ref_y = self.x, self.y
        self.stuck_ref_yaw = self.yaw

    # ------------------------------------------------------------------
    # Controle de missão
    # ------------------------------------------------------------------
    def advance_mission(self):
        self.mission_index += 1
        if self.mission_index >= len(self.mission_queue):
            self.state = State.MISSION_DONE
            self.get_logger().info('Missão concluída! Todos os alvos foram visitados '
                                    'e o robô retornou à posição inicial em cada etapa.')
            return

        self.current_label, self.goal_x, self.goal_y = self.mission_queue[self.mission_index]
        self.get_logger().info(f'Próximo destino: {self.current_label} '
                                f'({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.state = State.SEEKING


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
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