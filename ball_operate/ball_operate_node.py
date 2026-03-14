#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from imrc_messages.msg import BallInfo
from imrc_messages.msg import LedControl
from imrc_messages.msg import WallInfo
from std_msgs.msg import String
from imrc_messages.action import TiltAdjustment
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
from imrc_messages.action import Rotate

# 係数: 0.6 / 300^2 = 6.67e-6
K = 0.6 / (300 ** 2)

#14がマックス8
DX_TH = 4
#28がマックス　０　５２
DY_TH = 20

DEPTH_MIN = 41.5
DEPTH_MAX = 48.5

VEL = 0.04
FPS = 15

BACK_COUNT_MAX = 14
BACK_COUNT_MIN = 0

class BallOperate(Node):

    def __init__(self):
        super().__init__('ball_operate')

        #= cv_bridgeの初期化と画像保存用のディレクトリ作成 =
        self.bridge = CvBridge()
        self.image_save_dir = os.path.join(os.path.expanduser('~'), "haru26_ws",'images')
        os.makedirs(self.image_save_dir, exist_ok=True)
        self.image_counter = 0

        # ===== Publisher =====
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ball', 10)
        self.catch_pub = self.create_publisher(Bool, 'ball_catch', 10)
        self.led_pub = self.create_publisher(LedControl,'led_cmd',10)
        self.status_pub = self.create_publisher(Bool,'detect_ball_status',10)
        self.capture_pub = self.create_publisher(Bool, 'ball_capture', 10)
        self.cali_pub = self.create_publisher(Bool, 'ball_cali', 10)

        # ===== Subscriber =====
        self.create_subscription(BallInfo, 'ball_info', self.ball_cb, 10)
        self.create_subscription(String,'detect_ball_color',self.color_cb,10)
        self.create_subscription(Bool,'ball_operate_enable',self.enable_cb,10)
        self.create_subscription(Bool, 'ball_force_capture', self.ball_force_capture_cb, 10)
        self.create_subscription(Bool, 'ball_force_stop', self.ball_force_stop_cb, 10)
        self.create_subscription(Bool,"ball_back",self.ball_back_cb,10)
        self.create_subscription(Bool,"re_detect",self.re_detect_cb,10)
        self.create_subscription(WallInfo,"wall_raw",self.wall_filtered_cb,10)
        self.create_subscription(Twist,"cmd_vel_tilt_adjustment",self.tilt_adjustment_cb,10)
        self.create_subscription(Twist,"cmd_vel_rotate",self.rotate_cb,10)
        self.create_subscription(Bool,"cali_ok",self.cali_ok_cb,10)
        self.create_subscription(Image, 'ball_detector/raw_image', self.raw_image_cb, 10)
        
        # ===== Action Server =====
        self.adjustment_client = ActionClient(self, TiltAdjustment, 'TiltAdjustment')
        self.rotate_client = ActionClient(self, Rotate, 'Rotate')

        # ===== Timer =====
        self.create_timer(1.0 / FPS, self.timer_cb)

        # ===== 状態 =====
        self.enabled = False# アクション実行中か
        self.last_msg = None

        self.re_serch = False
        self.re_back = False

        self.back_count = 0
        self.retreating = False
        self.stopping = False
        self.stop_count = 0 
        self.status = 0 # 0:未検出　1:通常追従, 2:ボール捕獲
        self.his_status = 0

        self.reverse_operating = False
        self.adjusting = False


        # ===== 壁追従 PD制御 =====
        self.wall_target = 0.4

        self.kp_wall = 1.5
        self.kd_wall = 0.2

        self.prev_error_r = 0.0
        self.prev_error_l = 0.0

        self.msg_led = LedControl()


        self.cali_back_count = 10
        self.cali_back = False
        self.next = False

        self.one_rotate = False
        self.two_rotate = False
        self.rotating = False

    # ============================
    # 壁までの情報たちをうけるべ
    # ================================
    def wall_filtered_cb(self, msg: WallInfo):
        self.right_distance = msg.right_distance
        self.left_distance = msg.left_distance



    def color_cb(self,msg:String):
        self.ball_color = msg.data

    # ===============================
    #  ON / OFFにつかう
    # ===============================
    def enable_cb(self, msg: Bool):
        self._logger.info("探索が始まるよ！")
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

        self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=10.0)
        self.led_pub.publish(self.msg_led)


        self.status_pub.publish(Bool(data=False))

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

        self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=10.0)
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
            self.msg_led.blink_duration = 10.0 
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
            self.msg_led.blink_duration = 10.0 
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

    def re_detect_cb(self,msg:Bool):
        self.enabled = True
        self.re_serch = True

    # ===============================
    # adjustment_Actionの結果を受け取るコールバックだとおもう
    # ===============================
    def adjustment_response_callback(self, future):
        goal_handle = future.result()
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.adjustment_result_callback)

    def adjustment_result_callback(self, future):
        self.adjusting = False
        self.enabled = True
        self.get_logger().info("調整完了")

    # ===============================
    # cmd_vel_tilt_adjustmentをcmd_vel_ballに変換するコールバックだ
    # ===============================
    def tilt_adjustment_cb(self, msg: Twist):
        if self.adjusting:
            self.cmd_pub.publish(msg)

    # ===============================
    # rotate_Actionの結果を受け取るコールバックだとおもう
    # ===============================
    def rotate_response_callback(self, future):
        goal_handle = future.result()
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.rotate_result_callback)

    def rotate_result_callback(self, future):
        self.rotating = False
        self.enabled = True
        self.get_logger().info("回転完了")

    # ===============================
    # /cmd_vel_rotateをcmd_vel_ballに変換するコールバック
    # ===============================
    def rotate_cb(self, msg: Twist):
        if self.rotating:
            self.cmd_pub.publish(msg)

    # ===============================
    # raw_imageを受け取るコールバックだ
    # ===============================
    # def raw_image_cb(self, msg: Image):
    #     if not self.enabled:
    #         return
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         filename = os.path.join(self.image_save_dir, f'{self.image_counter:06d}.jpg')
    #         cv2.imwrite(filename, cv_image)
    #         self.image_counter += 1
    #     except Exception as e:
    #         self.get_logger().error(f'画像保存エラー: {e}')

    # ===============================
    # キャリブレーション完了を受け取るコールバック
    # ===============================
    def cali_ok_cb(self,msg: Bool):    
        self.enabled = True
        self.two_rotate = True
        if self.next:
            self. two_rotate = False
            self.next = False
            self.enabled = False
            self.status_pub.publish(Bool(data=False)) 

    # ===============================
    # 制御ループ
    # ===============================
    def timer_cb(self):
        twist = Twist()
        

        # ===== Action が来ていない or 終了後 =====
        if not self.enabled:
            if self.adjusting == False :
                if self.rotating == False:
                    self.cmd_pub.publish(Twist())  # 常に0
            
            return


        dx = self.last_msg.dx
        dy = self.last_msg.dy
        dep = self.last_msg.depth_cm

        # ==== 落ちたか落ちてないか判定 ====
        if self.re_serch:
            self.cali_back = True

            if (-(DX_TH + 10) <= dx <= (DX_TH + 10) and
                -(DY_TH + 8) <= dy <= (DY_TH + 8) and
                DEPTH_MIN - 1 <= dep <= DEPTH_MAX + 1
            ):
                self.re_serch = False
                #self.re_back = True
            else:
                self.next = True
                self.re_serch = False

        # キャリブレ前の動き
        if self.cali_back:    
            self._logger.info("キャリブレーション前の動きにはいったよ")
            self.cali_back = False
            self.enabled = False
            self.rotating = True

            goal_msg = Rotate.Goal()
            goal_msg.mode = "delta" 
            goal_msg.angle = 90.0
        
            self.rotate_client.wait_for_server()

            future = self.rotate_client.send_goal_async(goal_msg)
            future.add_done_callback(self.rotate_response_callback)
            self._logger.info("アクションのゴールを送ったよ")
            self.one_rotate = True
            return

        if self.one_rotate:
            self.cali_pub.publish(Bool(data=True))
            self.enabled = False
            self.one_rotate = False
            return
        
        if self.two_rotate:
            self._logger.info("２回めのアクションのゴールを送ったよ")
            self.enabled = False
            self.rotating = True
            self.two_rotate = False
            self.back_count = 0

            goal_msg = Rotate.Goal()
            goal_msg.mode = "delta"
            goal_msg.angle = -90.0
        
            self.rotate_client.wait_for_server()

            future = self.rotate_client.send_goal_async(goal_msg)
            future.add_done_callback(self.rotate_response_callback)
            return


        # ===== re後退フェーズ =====
        if self.re_back:
            if self.back_count > 0:
                twist.linear.x = -(VEL + 0.15)   
                self.back_count -= 1
                self.cmd_pub.publish(twist)
            else:
                self.re_back = False
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

                #self.msg_led = LedControl(led_brightness=0.0,led_index=5,led_color="RED",led_mode="apply",blink_duration=10.0)
                #self.led_pub.publish(self.msg_led)

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

            if self.reverse_operating == False:
                twist.linear.y = -0.4
            else:
                twist.linear.y = 0.4



            # ===== 右壁のやつ =====
            if self.reverse_operating == False:
                if self.right_distance <= 0.7:
                    
                    error = self.right_distance - self.wall_target
                    d_error = (error - self.prev_error_r) * FPS
                    u = self.kp_wall * error + self.kd_wall * d_error
                    twist.linear.y = -max(-0.4, min(0.4, u))
                    self.prev_error_r = error

                    # 停止したら方向反転
                    if self.right_distance <= 0.4:
                        self.reverse_operating = True
                        
                        self.enabled = False
                        self.adjusting = True

                        #direction_x: 'B', distance_x: 0.45, direction_y: 'R', distance_y: 0.37, angle_direction: 'B',angle: 0.0
                        goal_msg = TiltAdjustment.Goal()
                        goal_msg.direction_x = 'B'
                        goal_msg.distance_x = 0.45
                        goal_msg.direction_y = 'R'
                        goal_msg.distance_y = 0.37
                        goal_msg.angle_direction = 'B'
                        goal_msg.angle = 0.0
        
                        self.adjustment_client.wait_for_server()

                        future = self.adjustment_client.send_goal_async(goal_msg)
                        future.add_done_callback(self.adjustment_response_callback)

            # ===== 左壁noyatu =====
            if self.reverse_operating == True:
                if self.left_distance <= 0.7:

                    error = self.left_distance - self.wall_target
                    d_error = (error - self.prev_error_l) * FPS
                    u = self.kp_wall * error + self.kd_wall * d_error
                    twist.linear.y = max(-0.4, min(0.4, u))
                    self.prev_error_l = error

                    # 停止したら方向反転
                    if self.left_distance <= 0.4:
                        self.reverse_operating = False

                        self.enabled = False
                        self.adjusting = True
                        
                        #direction_y: 'L', distance_y: 0.30, angle_direction: 'L',angle: 0.0
                        goal_msg = TiltAdjustment.Goal()
                        goal_msg.direction_y = 'L'
                        goal_msg.distance_y = 0.30
                        goal_msg.angle_direction = 'L'
                        goal_msg.angle = 0.0
        
                        self.adjustment_client.wait_for_server()

                        future = self.adjustment_client.send_goal_async(goal_msg)
                        future.add_done_callback(self.adjustment_response_callback)
                        
            self.cmd_pub.publish(twist)
            return
        
        self.status = 1


        # ===== 通常追従 ===== 
        if self.reverse_operating == False:
            if self.right_distance <= 0.4:
                self.reverse_operating = True
        if self.reverse_operating == True:
            if self.left_distance <= 0.4:
                self.reverse_operating = False


        if dx < -DX_TH:
            twist.linear.y = max(min(K * abs(dx)**2, 0.6), 0.01)
        elif dx > DX_TH:
            twist.linear.y = min(max(-K * abs(dx)**2, -0.6), -0.01)

        if -DX_TH <= dx <= DX_TH:
            if dy < -DY_TH:

                twist.linear.x = min(0.002*abs(dy),0.5)
                self.back_count += 1
                self.back_count = max(BACK_COUNT_MIN, min(self.back_count, BACK_COUNT_MAX))

            elif dy > DY_TH:
                twist.linear.x = max(-0.002*abs(dy),-0.5)
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
            self.cmd_pub.publish(Twist()) #停止
            
            self.msg_led = LedControl(led_brightness=1.0,led_index=5,led_color="WHITE",led_mode="apply",blink_duration=10.0)
            self.led_pub.publish(self.msg_led)
            

            self.capture_pub.publish(Bool(data=True))
            self.enabled = False
            return
        
        elif (self.right_distance <= 0.5 and
              -(DX_TH) <= dx <= (DX_TH + 10) and
              -DY_TH <= dy <= DY_TH and
              DEPTH_MIN <= dep <= DEPTH_MAX
              ):
            self.get_logger().info(f"目標ボール捕捉 back_count={self.back_count}")
            self.cmd_pub.publish(Twist()) #停止
            
            self.msg_led = LedControl(led_brightness=1.0,led_index=5,led_color="WHITE",led_mode="apply",blink_duration=10.0)
            self.led_pub.publish(self.msg_led)
            

            self.capture_pub.publish(Bool(data=True))
            self.enabled = False
            return

        elif (self.left_distance <= 0.5 and
              -(DX_TH) <= dx <= (DX_TH + 10) and
              -DY_TH <= dy <= DY_TH and
              DEPTH_MIN <= dep <= DEPTH_MAX
              ):
            self.get_logger().info(f"目標ボール捕捉 back_count={self.back_count}")
            self.cmd_pub.publish(Twist()) #停止
            
            self.msg_led = LedControl(led_brightness=1.0,led_index=5,led_color="WHITE",led_mode="apply",blink_duration=10.0)
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