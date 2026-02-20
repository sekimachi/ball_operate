#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from imrc_messages.msg import BallInfo
from imrc_messages.msg import LedControl

DX_TH = 10
DY_TH = 10

DEPTH_MIN = 34.0
DEPTH_MAX = 37.0

VEL = 0.05
FPS = 15


class BallOperate(Node):

    def __init__(self):
        super().__init__('ball_operate')

        # ===== Publisher =====
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ball', 10)
        self.capture_pub = self.create_publisher(Bool, 'ball_capture', 10)
        self.led_pub = self.create_publisher(LedControl,'led_control',10)

        # ===== Subscriber =====
        self.create_subscription(BallInfo, 'ball_info', self.ball_cb, 10)
        
        self.create_subscription(Bool,'ball_operate_enable',self.enable_cb,10)
        # ===== Timer =====
        self.create_timer(1.0 / FPS, self.timer_cb)

        # ===== 状態 =====
        self.enabled = False# アクション実行中か
        self.last_msg = None

        self.back_count = 0
        self.retreating = False
        self.stopping = False
        self.stop_count = 0 
        self.status = 0 # 0:未検出　1:通常追従, 2:ボール捕獲

    # ===============================
    #  ON / OFFにつかうコールバック
    # ===============================
    def enable_cb(self, msg: Bool):
        self.enabled = msg.data


    # ===============================
    # ball_infoをうけとるコールバック
    # ===============================
    def ball_cb(self, msg: BallInfo):
        self.last_msg = msg

    # ===============================
    # 制御ループ
    # ===============================
    def timer_cb(self):
        twist = Twist()

        # ===== Action が来ていない or 終了後 =====
        if not self.enabled:
            self.cmd_pub.publish(Twist())  # 常に0
            return

        # ===== 停止フェーズ（6秒） =====
        if self.stopping:
            if self.stop_count > 0:
                self.stop_count -= 1
                self.cmd_pub.publish(Twist())  # 完全停止
            else:
                # 停止完了 → 後退開始
                self.stopping = False
                self.retreating = True
            return

        # ===== 後退フェーズ =====
        if self.retreating:
            if self.back_count > 0:
                twist.linear.x = -(VEL + 0.15)   # 後退
                self.back_count -= 1
                self.cmd_pub.publish(twist)
            else:
                # 後退完了
                self.retreating = False
                self.enabled = False
                self.status = 0
                self.capture_pub.publish(Bool(data=True))
                self.cmd_pub.publish(Twist())  # 完全停止
            return

        # ===== 未検出 =====
        if self.last_msg is None or not self.last_msg.detected:
            self.status = 0
            twist.linear.y = -(VEL + 0.15)
            self.cmd_pub.publish(twist)

            return
        
        self.status = 1
        dx = self.last_msg.dx
        dy = self.last_msg.dy
        dep = self.last_msg.depth_cm


        # ===== 通常追従 =====
        if dx < -DX_TH:
            twist.linear.y = VEL
        elif dx > DX_TH:
            twist.linear.y = -VEL

        if -DX_TH <= dx <= DX_TH:
            if dy < -DY_TH:
                twist.linear.x = VEL
                self.back_count += 1
            elif dy > DY_TH:
                twist.linear.x = -VEL
            else:
                if dep < DEPTH_MIN:
                    twist.linear.x = -VEL
                elif dep > DEPTH_MAX:
                    twist.linear.x = VEL
                    self.back_count += 1

        # ===== 捕捉判定 =====
        if (
            -DX_TH <= dx <= DX_TH and
            -DY_TH <= dy <= DY_TH and
            DEPTH_MIN <= dep <= DEPTH_MAX
        ):
            self.get_logger().info(f"目標ボール捕捉 back_count={self.back_count}")
            self.status = 0
            self.stopping = True
            self.stop_count = FPS * 6   # 6秒
            return


        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = BallOperate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
