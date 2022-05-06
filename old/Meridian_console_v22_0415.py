# #!/usr/bin/python3
# coding: UTF-8
# もしくは　#!/usr/bin/env python など環境に合わせて

#2022.02.05 UDP通信動作は安定。
#2022.02.05 COMMAND WINDOWのPOWERチェックボックスでサーボ電源ON
#2022.02.05 上記サーボ電源ON中にスライドバー操作でサーボ動作（ただしスライドバーが小さいため大まかな動作確認のみに利用可）
#2022.04.05 コードを少し整理整頓
#2022.04.14 各経路でのエラーの検知と表示の機能を搭載
#2022.04.14 ROSのパブリッシュボタンを追加。ROSのサブスクライブは未実装。

# 取扱説明書
# ・起動方法
# 当ファイルがあるディレクトリにて、ターミナルより
# python3 Meridian_console_v22_0415.py
# と入力して実行します。必要に応じてライブラリをpip3で追加してください。
# UDP_RESV_IP,UDP_SEND_IPについては予め調べスクリプト上で書き換えておく必要があります。
# UDP_RESV_IPはターミナルにてip a もしくはipconfig,ifconfig等で調べられます。
# UDP_SEND_IPはESP32の起動時にPCシリアルモニタ上に表示されます。
# ・画面について
# Command画面
#  POWER: 全サーボのパワーをオンオフします
#  Action: サインカーブの首振りモーションを送信します
#  ->ROS1: ROS1のjointデータをパブリッシュします（Rvisと連動できます）
#  <-ROS1: ROS1のサブスクライブですが未実装です。
#  Control Pad Monitor: リモコンの入力状態を標準化して表示します。
# Message画面
# IPと各経路のエラーカウント、エラー率、フレーム数、動作周波数を表示します
#  ResetCounter: カウンタの値をリセットするボタンです。
#  TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します（今はちょっと多めです。周波数を50Hzまで下げるとゼロになります。）
#  Sensor Monitor: MIUのデータを表示します。rol,pit,yawはセンサフュージョン値です。SetYawボタンでヨー軸の中央値をリセットできます。
#  Axis Monitor: 各サーボの値です。パワーオン時にはスライダでサーボを動かすことができます。

from ast import Pass
import numpy as np
import socket
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal
import time
#import random
import atexit
#import sys

#from re import I
import rospy
from sensor_msgs.msg import JointState
import struct

#定数
TITLE_VERSION="Meridian Console v22.0414" #DPGのウィンドウタイトル兼バージョン表示

UDP_RESV_IP="192.168.1.xx" #このPCのIPアドレス
UDP_RESV_PORT=22222 #受信ポート

UDP_SEND_IP="192.168.1.xx" #送信先のESP32のIPアドレス
UDP_SEND_PORT=22224 #送信ポート

MSG_SIZE = 90 #Meridim配列の長さ(デフォルトは90)
MSG_BUFF = MSG_SIZE * 2 #Meridim配列のバイト長さ

STEP = 0.02 #1フレームあたりに増加させる制御処理用の数値

#マスターコマンド用の定数(Meridim配列0番に格納する値)
CMD_SET_YAW_CENTER = 1002 #IMUのヨー軸センターリセットコマンド

#制御コマンド用フラグ等
flag_update_yaw_center = 0 #IMUのヨー軸センターリセットフラグ(python内部用)
flag_servo_power = 0 #全サーボのパワーオンオフフラグ
flag_send_data = 0 #状態データ送信モードのオンオフフラグ（サーボパワーオフでもデータ送信可能）
flag_send_motion = 0 #計算モーション送信のオンオフフラグ
flag_ros1_pub = 0 #ROS1のjoint_statesのパブリッシュ
flag_ros1_sub = 0 #ROS1のjoint_statesのサブスクライブ

#UDP用のsocket設定
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

#エラー集計表示用変数
loop_count = 0 #フレーム数のカウンタ
error_count_esp_to_pc = 0 #PCからESP32へのUDP送信でのエラー数
error_count_pc_to_esp = 0 #ESP32からPCへのUDP送信でのエラー数
error_count_esp_to_tsy = 0 #ESPからTeensyへのSPI通信でのエラー数
error_count_tsy_to_esp = 0 #TeensyからESP32へのSPI通信でのエラー数
error_count_tsy_skip = 0 #Teensyが受信したデータがクロックカウントスキップしていたか
error_count_pc_skip = 0 #PCが受信したデータがクロックカウントスキップしていたか
frame_clock_resv = 0 #今回受信したframe_clock
frame_clock_resv_past = 0#前回受信したframe_clock
start = time.time() # フレームレート計測用のタイマー初期値

#Meridim配列関連
r_meridim_disp=list(range(MSG_SIZE)) #Meridim配列の受信値short表示用
r_meridim_disp_char=list(range(MSG_SIZE*2)) #Meridim配列の受信値char表示用
s_meridim=[0]*MSG_SIZE #Meridim配列の送信値用
s_meridim_motion=[0]*MSG_SIZE #Meridim配列のPC側で作成したサーボ位置命令送信用

#メッセージ表示用
message0 = "This PC's IP adress is "+UDP_RESV_IP
message1 = ""
message2 = ""
message3 = ""

#モーション計算用変数
x = 0 #増分計算用 (STEPずつ)
y = 0 #増分計算用 (1ずつ)


#def udpresv():
#    pass

# デ ー タ の 送 受 信 #########################################################################
def main():

    global message0
    global message1
    global message2
    global message3
    global x
    global y

    while (True):
        
        message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:
                global loop_count
                global r_meridim_disp
                global r_meridim_disp_char
                global s_meridim_motion
                global error_count_pc_to_esp
                global error_count_esp_to_tsy
                global error_count_tsy_to_esp
                global error_count_esp_to_pc
                global error_count_tsy_skip
                global error_count_pc_skip
                global frame_clock_resv
                global frame_clock_resv_past
                global flag_servo_power
                global flag_ros1_pub
                global flag_ros1_sub

                loop_count += 1 #このpythonを起動してからのフレーム数をカウントアップ
                r_bin_data,addr = sock.recvfrom(1472)#UDPに受信したデータを転記
                r_meridim=struct.unpack('90h',r_bin_data)
                r_meridim_disp_char=struct.unpack('180b',r_bin_data)
                message1 = "UDP data receiving from "+UDP_SEND_IP

                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-1):
                    checksum[0] += r_meridim[i]
                    r_meridim_disp[i]=r_meridim[i]
                checksum[0] = ~checksum[0]

                temp = np.array([0], dtype=np.int16)

                # エラーフラグ各種のカウントアップ
                if checksum[0] == r_meridim[MSG_SIZE-1]:
                    if (r_meridim_disp[88] >> 14 & 1) == 1:#エラーフラグ14ビット目（ESP32のPCからのUDP受信のエラーフラグ）を調べる
                        error_count_pc_to_esp += 1
                    if (r_meridim_disp[88] >> 13 & 1) == 1:#エラーフラグ13ビット目（TeensyのESP32からのSPI受信のエラーフラグ）を調べる
                        error_count_esp_to_tsy += 1
                    if (r_meridim_disp[88] >> 12 & 1) == 1:#エラーフラグ12ビット目（ESPのTeensyからのSPI受信のエラーフラグ）を調べる
                        error_count_tsy_to_esp += 1
                    if (r_meridim_disp[88] >> 9 & 1) == 1:#エラーフラグ12ビット目（ESPのTeensyからのSPI受信のエラーフラグ）を調べる
                        error_count_tsy_skip += 1
                    temp[0] = r_meridim[88] & 0b0111111111111111 #エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を下げる
                else:
                    temp[0] = r_meridim[88] | 0b1000000000000000 #エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を上げる                    
                    error_count_esp_to_pc += 1

                frame_clock_resv_past = frame_clock_resv
                frame_clock_resv = r_meridim_disp[88] & 0b0000000011111111

                if((frame_clock_resv - frame_clock_resv_past == 1)or (frame_clock_resv - frame_clock_resv_past == -199)):
                    temp[0] &= 0b1111111011111111
                else:
                    if(loop_count >1):
                        print("Found data skipping on PC.")
                        error_count_pc_skip += 1
                        temp[0] |= 0b0000000100000000


                #time.sleep(1/1000) #少し休む場合


                # R O S 1 joint_statesのパブリッシュ =====================================================================================
                if (checksum[0] == r_meridim[MSG_SIZE-1]) and flag_ros1_pub:
                    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
                    rospy.init_node('joint_state_publisher_meridim')
                    rate = rospy.Rate(500) # 500hz
                    js_meridim = JointState()
                    js_meridim.header.stamp = rospy.Time.now()
                    js_meridim.name =     ['c_chest_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',                      'l_elbow_pitch',                    'l_hipjoint_yaw',                   'l_hipjoint_roll',                  'l_hipjoint_pitch',                 'l_knee_pitch',                     'l_ankle_pitch',                    'l_ankle_roll',                     'c_head_yaw',                       'r_shoulder_pitch',                  'r_shoulder_roll',                   'r_elbow_yaw',                      'r_elbow_pitch',                     'r_hipjoint_yaw',                    'r_hipjoint_roll',                  'r_hipjoint_pitch',                 'r_knee_pitch',                      'r_ankle_pitch',                     'r_ankle_roll']
                    js_meridim.position = [math.radians(r_meridim[21]/100), math.radians(r_meridim[23]/100), math.radians(r_meridim[25]/100), math.radians(r_meridim[27]/100), math.radians(r_meridim[29]/100), math.radians(r_meridim[31]/100), math.radians(r_meridim[33]/100), math.radians(r_meridim[35]/100), math.radians(r_meridim[37]/100), math.radians(r_meridim[39]/100), math.radians(r_meridim[41]/100), math.radians(r_meridim[51]/100), math.radians(r_meridim[53]/100), -math.radians(r_meridim[55]/100), -math.radians(r_meridim[57]/100), math.radians(r_meridim[59]/100), -math.radians(r_meridim[61]/100), -math.radians(r_meridim[63]/100), math.radians(r_meridim[65]/100), math.radians(r_meridim[67]/100),  math.radians(r_meridim[69]/100), -math.radians(r_meridim[71]/100)]
                    js_meridim.velocity = []
                    #js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    js_meridim.effort = []        
                    joint_pub.publish(js_meridim)
                    rate.sleep() 
                # ====================================================================================================

                # R O S 1 joint_statesのサブスクライブ ====================================================================================
                if (checksum[0] == r_meridim[MSG_SIZE-1]) and flag_ros1_sub:
                    #joint_sub = rospy.Subscriber('/joint_states_sub', JointState, queue_size=10)
                    #rospy.init_node('joint_state_subscriber_meridim', anonymous=True)
                    #rate = rospy.Rate(500) # 500hz
                    #js_meridim = JointState()
                    #js_meridim.header.stamp = rospy.Time.now()
                    #js_meridim.name =     ['c_chest_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',                      'l_elbow_pitch',                    'l_hipjoint_yaw',                   'l_hipjoint_roll',                  'l_hipjoint_pitch',                 'l_knee_pitch',                     'l_ankle_pitch',                    'l_ankle_roll',                     'c_head_yaw',                       'r_shoulder_pitch',                  'r_shoulder_roll',                   'r_elbow_yaw',                      'r_elbow_pitch',                     'r_hipjoint_yaw',                    'r_hipjoint_roll',                  'r_hipjoint_pitch',                 'r_knee_pitch',                      'r_ankle_pitch',                     'r_ankle_roll']
                    #js_meridim.position = [math.radians(r_meridim[21]/100), math.radians(r_meridim[23]/100), math.radians(r_meridim[25]/100), math.radians(r_meridim[27]/100), math.radians(r_meridim[29]/100), math.radians(r_meridim[31]/100), math.radians(r_meridim[33]/100), math.radians(r_meridim[35]/100), math.radians(r_meridim[37]/100), math.radians(r_meridim[39]/100), math.radians(r_meridim[41]/100), math.radians(r_meridim[51]/100), math.radians(r_meridim[53]/100), -math.radians(r_meridim[55]/100), -math.radians(r_meridim[57]/100), math.radians(r_meridim[59]/100), -math.radians(r_meridim[61]/100), -math.radians(r_meridim[63]/100), math.radians(r_meridim[65]/100), math.radians(r_meridim[67]/100),  math.radians(r_meridim[69]/100), -math.radians(r_meridim[71]/100)]
                    #js_meridim.velocity = []
                    #js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    #js_meridim.effort = []        
                    #joint_sub.subscribe(js_meridim)
                    #rate.sleep() 
                    pass
                # ====================================================================================================

                signal.signal(signal.SIGINT, signal.SIG_DFL)


                #if(frame_clock_resv_past != frame_clock_resv):

                #PC側サーボ位置発信用に最終サーボ情報をキープ
                if flag_servo_power == 2: #サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                    for i in  range(21,81,2):
                        s_meridim_motion[i] = r_meridim[i]
                    flag_servo_power = 1

                #送信データのベースを受信データのコピーで作成する
                s_meridim=[]
                s_meridim=list(r_meridim)

                #キープしたエラーフラグを格納
                s_meridim[88] = temp[0]

                #PC側でサーボ位置を計算制御する場合は以下でデータを作成
                if flag_send_motion  == 1: #
                    # xをフレームごとにカウントアップ
                    x += STEP
                    y += 1
                    if y>10000:
                            y = 0
                    if x>100:
                        x = 0
                    #print(np.sin(x),"  " ,x)
                    s_meridim_motion[51] = int(np.sin(x)*3000) #プラマイ10度の間でサインカーブを出力
                    print(s_meridim_motion[51])
                    s_meridim[1] = int(y) #チェック用。1から10000までのカウントアップをMerdim1番に入れる

                    #for i in  range(21,81,2):
                    #    s_meridim_motion[i] = r_meridim[i]

                #サーボオンオフフラグチェック：サーボオンフラグを格納
                if flag_servo_power > 0:
                    for i in  range(20,80,2):
                        s_meridim[i] = 1
                        s_meridim[i+1] = s_meridim_motion[i+1]
                else:
                    for i in  range(20,80,2):
                        s_meridim[i] = 0

                #マスターコマンドフラグチェック：ヨー軸センターリセットコマンドを格納
                global flag_update_yaw_center
                if (flag_update_yaw_center > 0):
                    flag_update_yaw_center -= 1 
                    s_meridim[0] = CMD_SET_YAW_CENTER
                    if (flag_update_yaw_center==0):
                        print("Send COMMAND 'Set Yaw Center.':["+str(CMD_SET_YAW_CENTER)+"]")

                #格納した送信データについてチェックサムを追加
                checksum[0] = 0
                checksum_int = 0
                for i in  range(MSG_SIZE-1):
                    checksum_int += s_meridim[i]
                checksum[0] = ~checksum_int
                s_meridim[MSG_SIZE-1]=checksum[0]

                time.sleep(2/1000) #少し休む場合

                #データをパックしてUDP送信
                s_bin_data=struct.pack('90h',*s_meridim)
                sock.sendto(s_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))


                #print("Frame "+str(int(frame_clock_resv - frame_clock_resv_past)))

                now = time.time()-start

                message2="ERROR COUNT ESP-PC:"+str("{:}".format(error_count_esp_to_pc))+\
                    " PC-ESP:"+str("{:}".format(error_count_pc_to_esp))+" ESP-TSY:"+str("{:}".format(error_count_esp_to_tsy))+" TsySKIP:"+\
                    str("{:}".format(error_count_tsy_skip))+" PcSKIP:"+str("{:}".format(error_count_pc_skip))+\
                    " Frames:"+str(loop_count)+"  "+str(int(loop_count/now))+"Hz"

                message3="ERROR RATE ESP-PC:"+str("{:.2%}".format(error_count_esp_to_pc/loop_count))+\
                    " PC-ESP:"+str("{:.2%}".format(error_count_pc_to_esp/loop_count))+" ESP-TSY:"+str("{:.2%}".format(error_count_esp_to_tsy/loop_count))


# 関 数 各 種 #########################################################################

def cleanup():#ctrl+cで終了したときにも確実にソケットを閉じる試み（いまのところ機能していない）
    print("Meridan_console quited.")
atexit.register(cleanup)

def set_servo_power():#チェックボックスに従いサーボパワーオンフラグをオンオフ
    global flag_servo_power
    if flag_servo_power == 0 :
        flag_servo_power = 2
        print("Servo Power ON")
    else:
        flag_servo_power = 0
        print("Servo Power OFF")

def set_data():#チェックボックスに従いデータ送信フラグをオンオフ
    global flag_send_data
    if flag_send_data == 0 :
        flag_send_data = 1
        print("Start sending Data.")
    else:
        flag_send_data = 0
        print("Quit sending Data.")

def set_action():#チェックボックスに従いアクション送信フラグをオンオフ
    global flag_send_motion
    if flag_send_motion == 0 :
        flag_send_motion = 1
        print("Start Motion Data Streaming.")
    else:
        flag_send_motion = 0
        print("Quit Motion Data Streaming.")

def ros1_pub():#チェックボックスに従いROS1パブリッシュフラグをオンオフ
    global flag_ros1_pub
    if flag_ros1_pub == 0 :
        flag_ros1_pub = 1
        print("Start publishing ROS1 joint_states.")
    else:
        flag_ros1_pub = 0
        print("Quit publishing ROS1 joint_states.")

def ros1_sub():#チェックボックスに従いROS1サブスクライブフラグをオンオフ
    global flag_ros1_sub
    if flag_ros1_sub == 0 :
        flag_ros1_sub = 1
        print("Start subscribing ROS1 joint_states.")
    else:
        flag_ros1_sub = 0
        print("Quit publishing ROS1 joint_states.")

def set_servo_angle(sender, app_data):#
    global s_meridim_motion
    if sender[3]=="L":
        s_meridim_motion[int(sender[4:6])*2+21] = int(app_data*100)
        print(f"L meri: {int(sender[4:6])*2+21}")
    if sender[3]=="R":
        s_meridim_motion[int(sender[4:6])*2+51] = int(app_data*100)
        print(f"R meri: {int(sender[4:6])*2+51}")

    print(f"sender is: {sender[3]}")
    print(f"sender is: {sender[4:6]}")
    print(f"app_data is: {int(app_data*100)}")
    print(f"motion is: {s_meridim_motion[int(sender[4:6])+21]}")


# dearpygui に よ る コ ン ソ ー ル 画 面 描 写 #########################################################################
def dpgrun():

    # dpg用関数 ==================================================
    def set_yaw_center():#IMUのヨー軸センターリセットフラグを10上げる（コマンドを10回送信する）
        global flag_update_yaw_center
        flag_update_yaw_center = 20

    def reset_counter():#カウンターのリセット
        global loop_count
        global error_count_pc_to_esp
        global error_count_esp_to_tsy
        global error_count_tsy_to_esp
        global error_count_esp_to_pc
        global error_count_tsy_skip
        global error_count_pc_skip
        global start
        loop_count = 1
        error_count_pc_to_esp = 0
        error_count_esp_to_tsy = 0
        error_count_tsy_to_esp = 0
        error_count_esp_to_pc = 0
        error_count_tsy_skip = 0
        error_count_pc_skip = 0
        start = time.time() 

    # dpg描画 ==================================================
    dpg.create_context()
    dpg.create_viewport(title=TITLE_VERSION, width=600, height=480)

    # （画面左上）サーボ位置モニタリング用のウィンドウ ==================================================
    with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
        with dpg.group(label='LeftSide'): 
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[10,35+i*20], width=80)
        with dpg.group(label='RightSide'):
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[135,35+i*20], width=80)

    # （画面下段）メッセージ表示用ウィンドウ（アドレス・通信エラー等） ==================================================
    with dpg.window(label="Messege", width=590, height=115,pos=[5,360]):
        dpg.add_button(label="ResetCounter",  callback=reset_counter, width =90, pos=[470,30])
        dpg.add_text(message0,tag="DispMessage0")
        dpg.add_text(message1,tag="DispMessage1")
        dpg.add_text(message2,tag="DispMessage2")
        dpg.add_text(message3,tag="DispMessage3")

    # （画面右側）センサー値モニタリング用ウィンドウ ==================================================
    with dpg.window(label="Sensor Monitor", width=335, height=175,pos=[260,5]):
        with dpg.group(label='LeftSide'): 
            dpg.add_slider_float(default_value=0, tag="mpu0", label="ac_x",max_value=327,min_value=-327,pos=[10,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu1", label="ac_y",max_value=327,min_value=-327,pos=[115,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu2", label="ac_z",max_value=327,min_value=-327,pos=[220,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu3", label="gr_x",max_value=327,min_value=-327,pos=[10,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu4", label="gr_y",max_value=327,min_value=-327,pos=[115,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu5", label="gr_z",max_value=327,min_value=-327,pos=[220,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu6", label="mg_x",max_value=327,min_value=-327,pos=[10,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu7", label="mg_y",max_value=327,min_value=-327,pos=[115,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu8", label="mg_z",max_value=327,min_value=-327,pos=[220,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu9", label="temp",max_value=327,min_value=-327,pos=[10,95], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu10", label="rol",max_value=327,min_value=-327,pos=[10,120], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu11", label="pit",max_value=327,min_value=-327,pos=[115,120], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu12", label="yaw",max_value=327,min_value=-327,pos=[220,120], width=60)
            dpg.add_button(label="SetYaw",  callback=set_yaw_center, width =50, pos=[270,148])

    # （画面右側中央段）コマンド送信/リモコン値表示用ウィンドウ ==================================================
    with dpg.window(label="Command", width=335, height=170,pos=[260,185]):
        dpg.add_checkbox(label="Power", tag="Power",  callback=set_servo_power, pos=[8,27])
        dpg.add_checkbox(label="Action", tag="Action",  callback=set_action, pos=[8,50])
        dpg.add_checkbox(label="->ROS1", tag="ROS1pub",  callback=ros1_pub, pos=[100,27])
        dpg.add_checkbox(label="<-ROS1", tag="ROS1sub",  callback=ros1_sub, pos=[100,50])
        dpg.add_text("Control Pad Monitor", pos=[10,100])
        dpg.add_text("button",tag="pad_button", pos=[170,100])
        dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx",max_value=127,min_value=-127, pos=[10,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly",max_value=127,min_value=-127, pos=[90,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx",max_value=127,min_value=-127, pos=[170,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry",max_value=127,min_value=-127, pos=[250,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v",max_value=255,min_value=0, pos=[90,140], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v",max_value=255,min_value=0, pos=[170,140], width=40)

    # dpg変数値の登録
    with dpg.value_registry():
        dpg.add_int_value(tag="button_data")

    dpg.setup_dearpygui()
    dpg.show_viewport()

    # dpg 描 画 内 容 の デ ー タ 更 新 ==================================================
    while dpg.is_dearpygui_running():

        #メッセージ欄の表示更新
        dpg.set_value("DispMessage0", message0) #メッセージ欄表示用
        dpg.set_value("DispMessage1", message1) #メッセージ欄表示用
        dpg.set_value("DispMessage2", message2) #メッセージ欄表示用
        dpg.set_value("DispMessage3", message3) #メッセージ欄表示用

        #サーボデータとIMUデータの表示更新
        for i in range(0, 15, 1):
            #global button
            idld = r_meridim_disp[21+i*2]
            idrd = r_meridim_disp[51+i*2]
            idsensor = r_meridim_disp[i+2]/10000
            dpg.set_value("ID L"+str(i), idld/100) #サーボIDと数値の表示L側
            dpg.set_value("ID R"+str(i), idrd/100) #サーボIDと数値の表示R側

            if i < 13: #IMUデータの更新
                if i < 11:
                    dpg.set_value("mpu"+str(i),idsensor)
                else:
                    dpg.set_value("mpu"+str(i),idsensor*100)

        #リモコンデータの表示更新
        dpg.set_value("pad_button", str(r_meridim_disp[80]))
        dpg.set_value("pad_Lx", r_meridim_disp_char[163])
        dpg.set_value("pad_Ly", r_meridim_disp_char[162])
        dpg.set_value("pad_Rx", r_meridim_disp_char[165])
        dpg.set_value("pad_Ry", r_meridim_disp_char[164])
        padL2val = (r_meridim_disp_char[167])
        if (padL2val<0):
            padL2val = 256+padL2val
        if (r_meridim_disp[80]&256==0):
            padL2val = 0
        padR2val = (r_meridim_disp_char[166])
        if (padR2val<0):
            padR2val = 256+padR2val
        if (r_meridim_disp[80]&512==0):
            padR2val = 0
        dpg.set_value("pad_L2v", padL2val)
        dpg.set_value("pad_R2v", padR2val)
        dpg.set_value("button_data", r_meridim_disp[80])

        #dpg表示更新処理
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

#スレッド2つで送受信と画面描写を並列処理
if __name__ == '__main__':
    thread1 = threading.Thread(target=dpgrun) #dearpyguiによるコンソール画面描写スレッド
    thread1.start()
#    thread2 = threading.Thread(target=udpresv)
#    thread2.start()
    #sys.exit(main())
    main()
