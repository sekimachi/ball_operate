#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from imrc_messages.msg import BallInfo
from imrc_messages.msg import LedControl
from std_msgs.msg import String

#14がマックス
DX_TH = 12
#28がマックス　０　５２
DY_TH = 25

DEPTH_MIN = 41.5
DEPTH_MAX = 51.0

VEL = 0.04
FPS = 15

BACK_COUNT_MAX = 10
BACK_COUNT_MIN = 0

class BallOperate(Node):

    def __init__(self):
        super().__init__('ball_operate')

        # ===== Publisher =====
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ball', 10)
        self.catch_pub = self.create_publisher(Bool, 'ball_catch', 10)
        self.led_pub = self.create_publisher(LedControl,'led_cmd',10)
        self.status_pub = self.create_publisher(Bool,'detect_ball_status',10)
        self.capture_pub = self.create_publisher(Bool, 'ball_capture', 10)

        # ===== Subscriber =====
        self.create_subscription(BallInfo, 'ball_info', self.ball_cb, 10)
        self.create_subscription(String,'detect_ball_color',self.color_cb,10)
        self.create_subscription(Bool,'ball_operate_enable',self.enable_cb,10)
        self.create_subscription(Bool, 'ball_force_capture', self.ball_force_capture_cb, 10)
        self.create_subscription(Bool, 'ball_force_stop', self.ball_force_stop_cb, 10)
        self.create_subscription(Bool,"ball_back",self.ball_back_cb,10)
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
        self.his_status = 0

        self.msg_led = LedControl()

    def color_cb(self,msg:String):
        self.ball_color = msg.data

    # ===============================
    #  ON / OFFにつかう
    # ===============================
    def enable_cb(self, msg: Bool):
        self.enabled = msg.data


    # ===============================
    # ball_infoをうけとるコールバック
    # ===============================
    def ball_cb(self, msg: BallInfo):
        self.last_msg = msg

    # ===============================
    # 強制捕捉トリガーをうけとるコールバック
    # ===============================
    def ball_force_capture_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("強制捕捉をしたよ")
            self.ball_force_capture()

    # ===============================
    # 強制終了トリガーをうけとるコールバック
    # ===============================
    def ball_force_stop_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info("強制終了をしたよ")
            self.ball_force_stop()

    # ===============================
    #  強制終了処理
    # ===============================
    def ball_force_stop(self):
        self.retreating = False
        self.stopping = False
        self.stop_count = 0
        self.back_count = 0
        self.status = 0
        self.enabled = False

        self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=250.0)
        self.led_pub.publish(self.msg_led)

        msg = Bool()
        msg.data = False
        self.status_pub.publish(msg)

        self.cmd_pub.publish(Twist())  # 完全停止


    # ===============================
    #  強制捕捉処理
    # ===============================
    def ball_force_capture(self):
        self.retreating = False
        self.stopping = False
        self.stop_count = 0
        self.back_count = 0
        self.status = 0
        self.enabled = False

        self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=250.0)
        self.led_pub.publish(self.msg_led)

        self.catch_pub.publish(Bool(data=True))
        self.cmd_pub.publish(Twist())  # 完全停止
        self.get_logger().info("強制捕捉完了")

    # ===============================
    # 
    # ==============================
    def LED_control(self,status):
        if status == 0:
            self.msg_led.led_brightness = 1.0   
            self.msg_led.led_index = 5          
            self.msg_led.led_mode = "apply"      
            self.msg_led.blink_duration = 1000.0 
            if self.ball_color == "赤":                
                self.msg_led.led_color = "RED"       
            elif self.ball_color == "青":
                self.msg_led.led_color = "BLUE"       
            elif self.ball_color == "黄":
                self.msg_led.led_color = "YELLOW"       

        if status == 1:
            self.msg_led.led_brightness = 1.0    
            self.msg_led.led_index = 5           
            self.msg_led.led_mode = "blink"      
            self.msg_led.blink_duration = 250.0 
            if self.ball_color == "赤":                
                self.msg_led.led_color = "RED"       
            elif self.ball_color == "青":
                self.msg_led.led_color = "BLUE"       
            elif self.ball_color == "黄":
                self.msg_led.led_color = "YELLOW"           
            
        self.led_pub.publish(self.msg_led)

    def ball_back_cb(self,msg:Bool):
        self.enabled = True
        self.retreating = True

    # ===============================
    # 制御ループ
    # ===============================
    def timer_cb(self):
        twist = Twist()
        

        # ===== Action が来ていない or 終了後 =====
        if not self.enabled:
            self.cmd_pub.publish(Twist())  # 常に0
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

                self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=250.0)
                self.led_pub.publish(self.msg_led)

                self.catch_pub.publish(Bool(data=True))
                self.cmd_pub.publish(Twist())  # 完全停止
            return

        # ==== LEDの制御 ====
        if self.status != self.his_status:
            self.his_status = self.status
            if self.status == 0:
                self.LED_control(0)
            if self.status == 1:
                self.LED_control(1)
            


        self.his_status = self.status

        # ===== 未検出 =====
        if self.last_msg is None or not self.last_msg.detected:
            self.status = 0
            twist.linear.y = -(VEL + 0.16)
            self.cmd_pub.publish(twist)
            return
        
        self.status = 1
        dx = self.last_msg.dx
        dy = self.last_msg.dy
        dep = self.last_msg.depth_cm


        # ===== 通常追従 ===== 
        if dx < -DX_TH:
            
            twist.linear.y = min(0.5,  VEL*abs(dx)*0.1)

        elif dx > DX_TH:
            twist.linear.y = max(-0.5, -VEL*abs(dx)*0.1)

        if -DX_TH <= dx <= DX_TH:
            if dy < -DY_TH:
                twist.linear.x = min(0.5, VEL*abs(dy)*0.1)
                self.back_count += 1
                self.back_count = max(BACK_COUNT_MIN, min(self.back_count, BACK_COUNT_MAX))

            elif dy > DY_TH:
                twist.linear.x = max(-0.5, -VEL*abs(dy)*0.1)
                self.back_count -= 1
                self.back_count = max(BACK_COUNT_MIN, min(self.back_count, BACK_COUNT_MAX))

            else:
                if dep < DEPTH_MIN:
                    twist.linear.x = -VEL
                    self.back_count -= 1
                    self.back_count = max(BACK_COUNT_MIN, min(self.back_count, BACK_COUNT_MAX))

                elif dep > DEPTH_MAX:
                    twist.linear.x = VEL
                    self.back_count += 1
                    self.back_count = max(BACK_COUNT_MIN, min(self.back_count, BACK_COUNT_MAX))

        # ===== 捕捉判定 =====
        if (-DX_TH <= dx <= DX_TH and
            -DY_TH <= dy <= DY_TH and
            DEPTH_MIN <= dep <= DEPTH_MAX
        ):
            self.get_logger().info(f"目標ボール捕捉 back_count={self.back_count}")
            
            
            self.msg_led = LedControl(led_brightness=1.0,led_index=5,led_color="WHITE",led_mode="apply",blink_duration=1000.0)
            self.led_pub.publish(self.msg_led)
            

            self.capture_pub.publish(Bool(data=True))
            self.enabled = False

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