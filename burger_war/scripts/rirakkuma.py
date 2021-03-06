#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------
# ファイル名 ：rirakkuma.py
# 機能概要  ：2019ROBOCON（予選用）
#           ・move_baseを使用
#           ・スタート/ゴール地点は"Start_Goal.csv"ファイルで設定する
# 作成日時  ：2019/08/19
# -----------------------------------------------------------------------

# Import
#   common
import rospy
import math
#   move_base
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
#   Twist
from geometry_msgs.msg import Twist
#   file
import csv  # csv file
import os   # file path
#   euler to quaternio
import tf
from geometry_msgs.msg import Quaternion

# Add ImageProcessing --- START ---
# use LaserScan
from sensor_msgs.msg import LaserScan

# use Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Image Process function
import imgProc        #function
from imgProc import * #class

# Add ImageProcessing --- END ---

import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#camera_fov = 50.0
#camera_width = 640.0


# PythonでEnum的なことを実現
class MoveState():
    STOP         = 0
    RUN          = 1
    DEFENSE      = 2

class RirakkumaBot():
    def __init__(self, bot_name="NoName"):
        ### Parameter Settings
        # bot name 
        self.name = bot_name

        # State
        self.move_state      = MoveState.STOP   # 移動状態           ：停止
        # CSV ファイルから取り出したデータ保存用リスト
        self.c_data          = []               # csvデータ
        self.c_data_cnt      = 0                # csvデータ順次取得のためのカウンタ
        # simple/goal用のシーケンス番号 ※これ無いとエラーになるため必要
        self.goal_seq_no     = 0

        ### Publisher を ROS Masterに登録
        # Velocity
        self.vel_pub         = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.pub_goal        = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        ### Subscriber を ROS Masterに登録
        self.sub_goal_result = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.result_callback, queue_size=1)
                # Add ImageProcessing --- START ---
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # for convert image topic to opencv obj
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        #self.image_sub = rospy.Subscriber('/red_bot/image_raw', Image, self.imageCallback)

        #cImgProc instance
        self.proc = cImgProc()
        # Add ImageProcessing --- END ---

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    def calcTwist_center(self, center, depth, S):
        #depth [m]
        if center != -1:
            val = int(center / 16) #centerを0-4の10段階に
            # --- 近距離 --------------------------------
            if 0.3 > depth:
            #if 100 < S:  
                x  =  -0.2
                th =  0.0
            # --- 中距離 --------------------------------
            elif 0.6 > depth:
                if   val == 4:
                    x  =  0.0
                    th = -0.2

                elif val == 3:
                    x  =  0.1
                    th = -0.1

                elif val == 2:
                    x = 0.0
                    th =  0.0

                elif val == 1:
                    x  =  0.1
                    th =  0.1

                else:
                    x  =  0.0
                    th =  0.2
            # --- 遠距離 ---------------------------------------                
            #elif 1.0 > depth:
            else :         
                if   val == 4:
                    x  =  0.0
                    th = -0.2

                elif val == 3:
                    x  =  0.1
                    th = -0.1

                elif val == 2:
                    x = 0.15
                    th =  0.0

                elif val == 1:
                    x  =  0.1
                    th =  0.1

                else:
                    x  =  0.0
                    th =  0.2
            # else:
            #     x=0.0
            #     th=0.0
        # --- no detect green
        else :
            x  = 0
            th = 0 

        # 更新
        print("blue detect x,th=", x, th)
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    # CSVファイルから座標を取得する関数
    def csv_data(self):
        # csvファイルをOpen
        csv_pass = os.path.dirname(__file__) + "/position_list.csv"
        csv_file = open(csv_pass, "r")
        # データ読み込み
        pos_data = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
        # 最初の一行をヘッダーとして取得
        header = next(pos_data)
        # 各行のデータを抜き出し
        for row in pos_data:
            # データ保存用のリストにcsvファイルから取得したデータを保存する
            # appendでリストに別のリストとして要素を追加する
            self.c_data.append(row)

    # "move_base/result" TopicをSubscribeしたときのコールバック関数
    # ゴール座標への到達を検知
    def result_callback(self,goal_result):
        print('result_callback')    # ★★デバッグ
        print(goal_result)          # ★★デバッグ
        if goal_result.status.status == 3:  # ゴールに到着 (★失敗時のAbort:4も対応する)
            if self.move_state == MoveState.RUN:
                self.move_state = MoveState.STOP         # 移動状態を更新：停止

    ### cmd_vel パラメータ設定＆Topic Publish関数 ※その場旋回用
    def vel_ctrl(self, line_x, line_y, ang_z):
        vel_msg = Twist()
        vel_msg.linear.x = line_x
        vel_msg.linear.y = line_y
        vel_msg.angular.z = ang_z
        self.vel_pub.publish(vel_msg)

    def orientstr_to_val(self,str_orient):
        ret_val = 0.0
        if str_orient == "up":
            ret_val = 0.0
        elif str_orient == "upright":
            ret_val = -math.pi/4
        elif str_orient == "right":
            ret_val = -math.pi/2
        elif str_orient == "downright":
            ret_val = -math.pi*3/4
        elif str_orient == "down":
            ret_val = math.pi
        elif str_orient == "downleft":
            ret_val = math.pi*3/4
        elif str_orient == "left":
            ret_val = math.pi/2
        elif str_orient == "upleft":
            ret_val = math.pi/4
        else:
            print("str_orient_error")
        return ret_val

    # "move_base_simple/goal"Topicのパラメータを設定し、Publishする関数 (引数はリスト型で渡す)
    def simple_goal_publish(self,pos_list):
        # Goal Setting
        goal = PoseStamped()
        goal.header.seq = self.goal_seq_no
        goal.header.frame_id = "/map"              # mapで座標系で指定する
        goal.header.stamp = rospy.Time.now()       # タイムスタンプは今の時間

        self.goal_seq_no += 1                      # シーケンス番号を更新

        # ** 位置座標
        goal.pose.position.x = float(pos_list[0])
        goal.pose.position.y = float(pos_list[1])
        goal.pose.position.z = 0
        # ** 回転方向
        # オイラー角をクォータニオンに変換・設定する
        # RESPECT @hotic06 オイラー角をクォータニオンに変換・設定する
        euler_val = self.orientstr_to_val(pos_list[2])
        quate = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_val)
        goal.pose.orientation.x = quate[0]
        goal.pose.orientation.y = quate[1]
        goal.pose.orientation.z = quate[2]
        goal.pose.orientation.w = quate[3]
        # debug
        print(goal)
        # 実際にTopicを配信する
        self.pub_goal.publish(goal)

    # ロボット動作のメイン処理
    def strategy(self):
        # 起動直後ウェイト
        rospy.sleep(1.0)  # 起動後、ウェイト（調整値）
        while not rospy.is_shutdown():
            if(self.proc.green_center != -1):
                print("detect green")
                self.client.cancel_goal()
                twist = self.calcTwist_center(self.proc.green_center, self.proc.green_center_depth, self.proc.green_center_S)
                print("#################### green_S_depth ####################")
                print(self.proc.green_center_S, "-", self.proc.green_center_depth)
                print("#######################################################")                
                self.vel_pub.publish(twist)
                print("snipe_enemy")
            elif self.move_state == MoveState.STOP:
                pos_info = self.c_data[self.c_data_cnt]
                if pos_info[3] == "way_point":
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.simple_goal_publish(pos_info)      # 座標は後ほどCSVから取得できるように
                    self.move_state = MoveState.RUN
                    print("run")
                elif pos_info[3] == "turn_r_45":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,-math.pi/4)            # その場でターン
                    print("r_45")
                elif pos_info[3] == "turn_r_90":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,-math.pi/2)            # その場でターン
                    print("r_90")
                elif pos_info[3] == "turn_l_45":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,math.pi/4)            # その場でターン
                    print("l_45")
                elif pos_info[3] == "turn_l_90":         # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,math.pi/2)            # その場でターン
                    print("l_90")
                elif pos_info[3] == "turn_wait":          # ※突貫コード
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.vel_ctrl(0,0,0)                 # その場でターンしない
                    print("turn_wait")
                else:
                    self.move_state = MoveState.DEFENSE
                    pass
            else:
                pass

            print('move_state')  # ★★デバッグ
            print(self.move_state)
            rospy.sleep(1)  # 1秒Wait

    # Add ImageProcessing --- START ---
    def lidarCallback(self, data):
        self.scan = data

    # camera image call back sample
    def imageCallback(self, data):
        # comvert image topic to opencv object
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # image processing
        # liderCallbackより先にimageCallbackがコールされIndexError例外に対応
        try:
            self.proc.imageProcess1(self.img, self.scan)
            #print('cwd=', self.proc.cwd)
        except IndexError as e:
            print(e)
            return

        # Show camera window
        if self.proc.debug_view == 1:           
            cv2.imshow("Camera", self.proc.img_div2)            
            cv2.waitKey(1)

        # Show debug window
        if self.proc.debug_view == 2:
            #cv2.imshow("rila", self.proc.rila_img)
            #cv2.imshow("div2", self.proc.img_div2)            
            #cv2.imshow("div8", self.proc.img_div8)
            #cv2.imshow("red", self.proc.red_img)
            #cv2.imshow("green", self.proc.green_img)
            #cv2.imshow("blue", self.proc.blue_img)                        
            #cv2.imshow("Camera", self.proc.img)            
            cv2.imshow("debug1", self.proc.debug1_img)                        
            # --- add T.Ishigami 2020.03.15 22:40 ---
            # Add window position for FullHD
            cv2.moveWindow("debug1", 0, 820)

            cv2.waitKey(1)
        # green_index = self.proc.green_center
        # if green_index != -1:
        #     green_distance = self.proc.depth_img.item(0,green_index,0)
        # else:
        #     green_distance = 0

    # Add ImageProcessing --- END ---    

if __name__ == "__main__":
    rospy.init_node('rirakkuma_node')
    bot = RirakkumaBot('rirakkuma')
    bot.csv_data()
    bot.strategy()