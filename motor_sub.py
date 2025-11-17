#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
from collections import deque
import time

class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # --------------------------
        # 센서 구독
        # --------------------------
        self.create_subscription(Float32, '/angle', self.angle_callback, 10)
        self.create_subscription(Float32, '/deg', self.deg_callback, 10)
        self.create_subscription(Bool, '/deg_valid', self.deg_valid_callback, 10)

        # --------------------------
        # 속도/주행 명령
        # --------------------------
        self.create_subscription(String, '/speed_cmd', self.speed_cmd_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # --------------------------
        # 차선 변경 명령
        # --------------------------
        self.create_subscription(String, '/lane_change_cmd', self.lane_change_callback, 10)

        # --------------------------
        # 퍼블리셔
        # --------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --------------------------
        # 상태 변수
        # --------------------------
        self.angle_data = 0.0
        self.deg_data = 0.0
        self.deg_valid = False

        self.linear_speed = 0.08
        self.target_speed = 0.08
        self.saved_speed = 0.08
        self.speed_step = 0.01

        self.k_angle = 0.002
        self.k_deg = 0.008
        self.max_angular = 16.0

        # 딜레이 큐
        self.z_queue = deque(maxlen=3)
        self.z_filtered = 0.0

        # 상태 플래그
        self.stop_flag = False
        self.emergency_flag = False

        # 강제 조향
        self.force_turn = False
        self.force_turn_end_time = 0.0
        self.force_turn_z = 0.0

        # --------------------------
        # 제어 루프
        # --------------------------
        self.create_timer(0.03, self.control_callback)

    # --------------------------
    # 센서 콜백
    # --------------------------
    def angle_callback(self, msg):
        self.angle_data = msg.data

    def deg_callback(self, msg):
        self.deg_data = msg.data

    def deg_valid_callback(self, msg):
        self.deg_valid = msg.data

    # --------------------------
    # 속도 및 주행 명령
    # --------------------------
    def speed_cmd_callback(self, msg: String):
        cmd = msg.data.lower()
        if cmd == "up":
            self.target_speed = min(self.target_speed + 0.02, 0.25)
            self.saved_speed = self.target_speed
            self.stop_flag = False
            self.get_logger().info(f"⏫ 속도 증가 → {self.target_speed:.3f}")

        elif cmd == "down":
            self.target_speed = max(self.target_speed - 0.02, 0.02)
            self.saved_speed = self.target_speed
            self.stop_flag = False
            self.get_logger().info(f"⏬ 속도 감소 → {self.target_speed:.3f}")

        elif cmd == "stop":
            self.stop_flag = True
            self.target_speed = 0.0
            self.get_logger().info("🛑 정지 명령 → 감속 중...")

        elif cmd == "start":
            self.stop_flag = False
            self.target_speed = self.saved_speed
            self.get_logger().info(f"▶️ 주행 시작 → 속도 {self.target_speed:.3f}")

    # --------------------------
    # 긴급 정지
    # --------------------------
    def emergency_callback(self, msg: Bool):
        if msg.data:
            self.emergency_flag = True
            self.saved_speed = self.target_speed
            self.target_speed = 0.0
            self.get_logger().warn("🔥 긴급정지! 즉시 정지 중...")
        else:
            if self.emergency_flag:
                self.emergency_flag = False
                self.stop_flag = False
                self.target_speed = self.saved_speed
                self.get_logger().info(f"🔄 긴급정지 해제 → 속도 복원 {self.target_speed:.3f}")

    # --------------------------
    # 차선 변경
    # --------------------------
    def lane_change_callback(self, msg):
        key = msg.data.lower().strip()
        if key == 'left':
            self.force_turn = True
            self.force_turn_z = 0.9
            self.force_turn_end_time = time.time() + 1.2
            self.get_logger().info("⬅️ FORCE LEFT TURN")
        elif key == 'right':
            self.force_turn = True
            self.force_turn_z = -0.9
            self.force_turn_end_time = time.time() + 1.2
            self.get_logger().info("➡️ FORCE RIGHT TURN")

    # --------------------------
    # 메인 제어 루프
    # --------------------------
    def control_callback(self):
        now = time.time()

        # 속도 ramping
        if self.linear_speed < self.target_speed:
            self.linear_speed += self.speed_step
            self.linear_speed = min(self.linear_speed, self.target_speed)
        elif self.linear_speed > self.target_speed:
            self.linear_speed -= self.speed_step
            self.linear_speed = max(self.linear_speed, self.target_speed)

        # 강제 조향 모드
        if self.force_turn:
            if now < self.force_turn_end_time:
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.force_turn_z
                self.cmd_pub.publish(twist)
                return
            else:
                self.force_turn = False

        # 일반 조향 계산
        if self.deg_valid:
            z_now = -self.k_deg * self.deg_data
        else:
            a = 0.0 if abs(self.angle_data) < 3 else self.angle_data
            a = max(min(a, 120), -120)
            z_now = -self.k_angle * a

        z_now = max(min(z_now, self.max_angular), -self.max_angular)

        # 딜레이 큐 적용
        self.z_queue.append(z_now)
        z = self.z_queue[0] if len(self.z_queue) >= 3 else z_now

        # Twist 발행
        twist = Twist()
        twist.linear.x = float(self.linear_speed)
        twist.angular.z = float(z)
        self.cmd_pub.publish(twist)

# --------------------------
# main
# --------------------------
def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
