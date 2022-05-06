# #!/usr/bin/python3
# coding: UTF-8
# もしくは　#!/usr/bin/env python など環境に合わせて

# Izumi Ninagawa & Meridian project
# 2022.02.05 UDP通信動作は安定。
# 2022.02.05 COMMAND WINDOWのPOWERチェックボックスでサーボ電源ON
# 2022.02.05 上記サーボ電源ON中にスライドバー操作でサーボ動作（ただしスライドバーが小さいため大まかな動作確認のみに利用可）
# 2022.04.05 コードを少し整理整頓
# 2022.04.14 各経路でのエラーの検知と表示の機能を搭載
# 2022.04.18 intel Mac用にdearpygui処理をmainスレッドに変更。M1 Macではdearpyguiが未対応のため動かないはず。
# 2022.04.25 command欄にパワーオンや送受信の経路スイッチャを追加。（ROS1の受信は機能として未テスト）
# 2022.04.25 virtualのボタンも機能未実装（ハードウェアのセンサ信号などを仮想化した環境ものと通信できるようにする想定）

# 取扱説明書
# ・起動方法
# 当ファイルがあるディレクトリにて、ターミナルより
# python3 Meridian_console.py
# と入力して実行します。必要に応じてライブラリをpip3で追加してください。
# UDP_RESV_IP,UDP_SEND_IPについては予め調べスクリプト上で書き換えておく必要があります。
# UDP_RESV_IPはターミナルにてip a もしくはipconfig,ifconfig等で調べられます。
# UDP_SEND_IPはESP32の起動時にPCシリアルモニタ上に表示されます。
# ・画面について
# Command画面
# POWER: 全サーボのパワーをオンオフします
# Action: サインカーブの首振りモーションを送信します
# ->ROS1: ROS1のjointデータをパブリッシュします（Rvisと連動できます）
# <-ROS1: ROS1のサブスクライブですが未実装です。
# Control Pad Monitor: リモコンの入力状態を標準化して表示します。
# Message画面
# IPと各経路のエラーカウント、エラー率、フレーム数、動作周波数を表示します
# ResetCounter: カウンタの値をリセットするボタンです。
# TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します（今はちょっと多めです。周波数を50Hzまで下げるとゼロになります。）
# Sensor Monitor: MIUのデータを表示します。rol,pit,yawはセンサフュージョン値です。SetYawボタンでヨー軸の中央値をリセットできます。
# Axis Monitor: 各サーボの値です。パワーオン時にはスライダでサーボを動かすことができます。
# 100Hz動作時にTeensyの受信スキップ回数が5%ほど検出されるのは、現在の仕様で正常な動作です。

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
import atexit
import struct

#ROS 未搭載マシンの場合は、下記の２行を#でコメントアウトしてください。
import rospy
from sensor_msgs.msg import JointState

#定数
TITLE_VERSION="Meridian Console v22.0425" #DPGのウィンドウタイトル兼バージョン表示

UDP_RESV_IP="192.168.xx.xx" #このPCのIPアドレス
UDP_RESV_PORT=22222 #受信ポート

UDP_SEND_IP="192.168.xx.xx" #送信先のESP32のIPアドレス
UDP_SEND_PORT=22224 #送信ポート

MSG_SIZE = 90 #Meridim配列の長さ(デフォルトは90)
MSG_BUFF = MSG_SIZE * 2 #Meridim配列のバイト長さ

STEP = 0.02 #1フレームあたりに増加させる制御処理用の数値

#マスターコマンド用の定数(Meridim配列0番に格納する値)
CMD_SET_YAW_CENTER = 1002 #IMUのヨー軸センターリセットコマンド

#制御コマンド用フラグ等
flag_update_yaw_center = 0 #IMUのヨー軸センターリセットフラグ(python内部用)
flag_servo_power = 0 #全サーボのパワーオンオフフラグ
flag_resv_data = 0 #ESP32からの状態データの受信のオンオフフラグ（モーション送信時のシミュレーション空間用として）
flag_send_data = 0 #ESP32への状態データの送信のオンオフフラグ（サーボパワーオフでもデータ送信可能にすべく）
flag_send_virtual = 0 #ハードウェアを接続しないで動作させる場合のバーチャルハードのオンオフフラグ
flag_send_motion = 0 #計算モーション送信のオンオフフラグ
flag_demo_action = 0 #デモ/テスト用の計算モーション送信のオンオフフラグ
flag_ros1_pub = 0 #ROS1のjoint_statesのパブリッシュ
flag_ros1_sub = 0 #ROS1のjoint_statesのサブスクライブ
flag_ros1 = 0 #ROS1の起動init（初回のみ）

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
error_count_esp_skip = 0 #ESPが受信したデータがクロックカウントスキップしていたか
error_count_pc_skip = 0 #PCが受信したデータがクロックカウントスキップしていたか
frame_sync_s = 0 #送信するframe_sync_r(0-199)
frame_sync_r_expect = 0 #毎フレームカウントし、受信カウントと比較(0-199)
frame_sync_r_resv = 0 #今回受信したframe_sync_r
start = time.time() # フレームレート計測用のタイマー初期値

#Meridim配列関連
#r_meridim_disp=list(range(MSG_SIZE)) #Meridim配列の受信値short表示用
r_meridim_char=list(range(MSG_SIZE*2)) #Meridim配列の受信値char表示用
r_meridim=[0]*MSG_SIZE #Meridim配列の送信値用
s_meridim=[0]*MSG_SIZE #Meridim配列の送信値用
s_meridim_js_sub=[0]*MSG_SIZE #ROSからサブスクライブしたサーボ位置情報の格納用Meridim配列
s_meridim_motion=[0]*MSG_SIZE #Meridim配列のPC側で作成したサーボ位置命令送信用
s_meridim_motion_keep=[0]*MSG_SIZE #Meridim配列のパワーオン時の位置キープ用

#メッセージ表示用
message0 = "This PC's IP adress is "+UDP_RESV_IP
message1 = ""
message2 = ""
message3 = ""
message4 = ""

#モーション計算用変数
x = 0 #増分計算用 (STEPずつ)
y = 0 #増分計算用 (1ずつ)

jspn = list(range(30)) #サーボ角度のROS joint_states変換用の回転方向順逆補正
jspn[0] = 1 #頭ヨー
jspn[1] = 1 #左肩ピッチ
jspn[2] = 1 #左肩ロール
jspn[3] = 1 #左肘ヨー
jspn[4] = 1 #左肘ピッチ
jspn[5] = 1 #左股ヨー
jspn[6] = 1 #左股ロール
jspn[7] = 1 #左股ピッチ
jspn[8] = 1 #左膝ピッチ
jspn[9] = 1 #左足首ピッチ
jspn[10] = 1 #左足首ロール
jspn[11] = 1 #予備
jspn[12] = 1 #予備
jspn[13] = 1 #予備
jspn[14] = 1 #予備
jspn[15] = 1 #腰ヨー
jspn[16] = 1 #右肩ピッチ
jspn[17] = -1 #右肩ロール
jspn[18] = -1 #右肘ヨー
jspn[19] = 1 #右肘ピッチ
jspn[20] = -1 #右股ヨー
jspn[21] = -1 #右股ロール
jspn[22] = 1 #右股ピッチ
jspn[23] = 1 #右膝ピッチ
jspn[24] = 1 #右足首ピッチ
jspn[25] = -1 #右足首ロール
jspn[26] = 1 #予備
jspn[27] = 1 #予備
jspn[28] = 1 #予備
jspn[29] = 1 #予備



#def udpresv():
#    pass

##################################################################################################################################################
# デ ー タ の 送 受 信 ###########################################################################################################################
##################################################################################################################################################

def meridian_loop():

    global message0
    global message1
    global message2
    global message3
    global message4
    global x
    global y

    while (True):
        print("checksss")
        
        message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:
                global loop_count
                global r_meridim
                global r_meridim_char
                global s_meridim_motion
                global error_count_pc_to_esp
                global error_count_esp_to_tsy
                global error_count_tsy_to_esp
                global error_count_esp_to_pc
                global error_count_esp_skip
                global error_count_tsy_skip
                global error_count_pc_skip
                global frame_sync_s
                global frame_sync_r_expect
                global frame_sync_r_resv
                global flag_servo_power
                global flag_demo_action
                global flag_send_data
                global flag_resv_data
                global flag_ros1_pub
                global flag_ros1_sub

                loop_count += 1 #このpythonを起動してからのフレーム数をカウントアップ

                r_bin_data,addr = sock.recvfrom(1472) #UDPに受信したデータを転記してキープ
                r_meridim=struct.unpack('90h',r_bin_data) #short型のMeridim90を作成
                r_meridim_char=struct.unpack('180b',r_bin_data) #読み取り用のchar型Meridim(180)を作成
                message1 = "UDP data receiving from "+UDP_SEND_IP #受信中のメッセージ表示

                #受信データに対するチェックサムの実行
                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-1):
                    checksum[0] += r_meridim[i]
                checksum[0] = ~checksum[0]

                # エラーフラグ各種のカウントアップ
                temp = np.array([0], dtype=np.int16)#short型のビットをpythonで扱うためのnumpy配列テンポラリ変数
                if checksum[0] == r_meridim[MSG_SIZE-1]:
                    if (r_meridim[88] >> 14 & 1) == 1:#エラーフラグ14ビット目（ESP32のPCからのUDP受信のエラーフラグ）を調べる
                        error_count_pc_to_esp += 1
                    if (r_meridim[88] >> 13 & 1) == 1:#エラーフラグ13ビット目（TeensyのESP32からのSPI受信のエラーフラグ）を調べる
                        error_count_esp_to_tsy += 1
                    if (r_meridim[88] >> 12 & 1) == 1:#エラーフラグ12ビット目（ESPのTeensyからのSPI受信のエラーフラグ）を調べる
                        error_count_tsy_to_esp += 1
                    if (r_meridim[88] >> 10 & 1) == 1:#エラーフラグ10ビット目（ESPのPCからのUDP受信のフレーム連番スキップフラグ）を調べる
                        error_count_esp_skip += 1
                    if (r_meridim[88] >> 9 & 1) == 1:#エラーフラグ9ビット目（TeensyのESP経由のPCから受信のフレーム連番スキップフラグ）を調べる
                        error_count_tsy_skip += 1 
                    temp[0] = r_meridim[88] & 0b0111111111111111 #エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を下げる
                else:
                    temp[0] = r_meridim[88] | 0b1000000000000000 #エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を上げる                    
                    error_count_esp_to_pc += 1 #PCのUDP受信エラーをカウントアップ

                #受信予測用のカウントアップ
                frame_sync_r_expect += 1
                if frame_sync_r_expect > 199:
                    frame_sync_r_expect = 0

                #フレームスキップチェック用のカウントの受信と処理
                frame_sync_r_resv = r_meridim[88] & 0b0000000011111111 #カウントを受信

                if(frame_sync_r_resv == frame_sync_r_expect): #受信したカウントが予想通りであればスキップなし
                    temp[0] &= 0b1111111011111111 #PCのESP経由Teensyからの連番スキップフラグを下げる
                else:
                    #print("Found data unsync on PC.")
                    temp[0] |= 0b0000000100000000 #PCのESP経由Teensyからの連番スキップフラグを上げる
                    frame_sync_r_expect = frame_sync_r_resv #受信カウントの方が多ければズレを検出し、追いつく
                    error_count_pc_skip += 1 #スキップカウントをプラス

                #送信用クロックの準備
                frame_sync_s += 1 #送信用のframe_sync_sをカウントアップ
                if frame_sync_s > 199:
                    frame_sync_s = 0
                temp[0] &= 0b1111111100000000 #下位8ビットをクリア
                temp[0] += frame_sync_s #下位8ビットにframe_sync_sカウントアップを格納

                #PC側サーボ位置発信用に最終サーボ情報をキープ
                if flag_servo_power == 2: #サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                    for i in  range(21,81,2):
                        s_meridim_motion[i] = r_meridim[i]
                        s_meridim_motion_keep[i] = r_meridim[i]
                    flag_servo_power = 1

        # 送信用のモーションを作成（①受信値そのまま、②ROSサブスク反映、③計算モーション）
                if checksum[0] == r_meridim[MSG_SIZE-1]:#受信成功時はデータ更新
                    s_meridim=[] #データのクリア
                    s_meridim=list(r_meridim)

                #①受信値そのままの場合：送信データのベースを受信データのコピーで作成
                if flag_servo_power:#サーボパワーオン時は、電源入力時に保持した値を固定で流す（ハウリング的なサーボ位置ズレの増幅を防止）
                        for i in  range(21,81,2):
                            s_meridim[i] = s_meridim_motion_keep[i]
                else:
                    if flag_resv_data:
                        for i in  range(21,81,2):#受信サーボ値を書き込みモーションのベースとして一旦キープ
                            s_meridim_motion[i] = r_meridim[i]

                #②サーボ位置にROSのサブスクライブを反映させる場合にはここでデータを作成★★
                if flag_ros1_sub:
                    for i in  range(11):
                        s_meridim_motion[21+i*2] = s_meridim_js_sub[21+i*2]
                        s_meridim_motion[51+i*2] = s_meridim_js_sub[51+i*2]

                #③サーボ位置をここで計算制御する場合は以下でデータを作成(まずはデモモーションのみで運用テスト)
                if flag_demo_action: #
                    # xをフレームごとにカウントアップ
                    x += STEP
                    if x>100:
                        x = 0
                    s_meridim_motion[51] = int(np.sin(x)*3000) #プラマイ10度の間で頭ヨー軸のみにサインカーブを出力

        # データを送信Meridim配列に格納

                #サーボオンオフフラグチェック：サーボオンフラグを格納
                if flag_servo_power > 0:
                    for i in  range(20,80,2):
                        s_meridim[i] = 1
                else:
                    for i in  range(20,80,2):
                        s_meridim[i] = 0

                #PC側発行のサーボ位置を格納
                if flag_send_data:
                    for i in  range(21,81,2):
                        s_meridim[i] = s_meridim_motion[i]

                #マスターコマンドフラグチェック：ヨー軸センターリセットコマンドを格納
                global flag_update_yaw_center
                if (flag_update_yaw_center > 0):
                    flag_update_yaw_center -= 1 
                    s_meridim[0] = CMD_SET_YAW_CENTER
                    if (flag_update_yaw_center==0):
                        print("Send COMMAND 'Set Yaw Center.':["+str(CMD_SET_YAW_CENTER)+"]")

                #キープしたエラーフラグ/送信クロックを格納
                s_meridim[88] = temp[0]

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

                #print("Frame "+str(int(frame_sync_r_resv - frame_sync_r_resv_past)))

                now = time.time()-start

                message2="ERROR COUNT ESP-PC:"+str("{:}".format(error_count_esp_to_pc))+\
                    " PC-ESP:"+str("{:}".format(error_count_pc_to_esp))+" ESP-TSY:"+str("{:}".format(error_count_esp_to_tsy))

                message3="ERROR RATE ESP-PC:"+str("{:.2%}".format(error_count_esp_to_pc/loop_count))+\
                    " PC-ESP:"+str("{:.2%}".format(error_count_pc_to_esp/loop_count))+" ESP-TSY:"+str("{:.2%}".format(error_count_esp_to_tsy/loop_count))

                message4="SKIP COUNT TsySKIP:"+\
                    str("{:}".format(error_count_tsy_skip))+" ESPSKIP:"+str("{:}".format(error_count_esp_skip))+" PcSKIP:"+str("{:}".format(error_count_pc_skip))+\
                    " Frames:"+str(loop_count)+"  "+str(int(loop_count/now))+"Hz"

##################################################################################################################################################
# 関 数 各 種 ####################################################################################################################################
##################################################################################################################################################

def cleanup():#ctrl+cで終了したときにも確実にソケットを閉じる試み（いまのところ機能していない）
    print("Meridan_console quited.")
atexit.register(cleanup)#この行は機能しているかどうかわからない

def set_servo_power():#チェックボックスに従いサーボパワーオンフラグをオンオフ
    global flag_servo_power
    if flag_servo_power == 0 :
        flag_servo_power = 2
        print("Servo Power ON")
    else:
        flag_servo_power = 0
        print("Servo Power OFF")

def set_demo_action():#チェックボックスに従いアクション送信フラグをオンオフ
    global flag_demo_action
    if flag_demo_action == 0 :
        flag_demo_action = 1
        print("Start DEMO motion data streaming.")
    else:
        flag_demo_action = 0
        print("Quit DEMO motion data streaming.")

def set_resv_data():#チェックボックスに従いデータ送信フラグをオンオフ
    global flag_resv_data
    if flag_resv_data == 0 :
        flag_resv_data = 1
        print("Start receiving data from ESP32.")
    else:
        flag_resv_data = 0
        print("Quit receiving data from ESP32.")

def set_send_data():#チェックボックスに従いデータ送信フラグをオンオフ
    global flag_send_data
    if flag_send_data == 0 :
        flag_send_data = 1
        print("Start sending data to ESP32.")
    else:
        flag_send_data = 0
        print("Quit sending data to ESP32.")

def set_send_virtual():#チェックボックスに従いデータ送信フラグをオンオフ
    global flag_send_virtual
    if flag_send_virtual == 0 :
        flag_send_virtual = 1
        print("Start noting. Virtual-Hard Uninplemented.")
    else:
        flag_send_virtual = 0
        print("Quit nothing. Virtual-Hard Uninplemented")

def ros1_pub():#チェックボックスに従いROS1パブリッシュフラグをオンオフ
    #print("ROS1 is not available.")
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

def set_servo_angle(channel, app_data):#
    global s_meridim_motion
    if channel[3]=="L":
        s_meridim_motion[int(channel[4:6])*2+21] = int(app_data*100)
        print(f"L meri: {int(channel[4:6])*2+21}")
    if channel[3]=="R":
        s_meridim_motion[int(channel[4:6])*2+51] = int(app_data*100)
        print(f"R meri: {int(channel[4:6])*2+51}")

    print(f"channel is: {channel[3]}")
    print(f"channel is: {channel[4:6]}")
    print(f"app_data is: {int(app_data*100)}")
    print(f"motion is: {s_meridim_motion[int(channel[4:6])+21]}")

def callback(JointState):
    global s_meridim_js_sub
    global jspn
    for i in  range(11):
        s_meridim_js_sub[21+i*2]=round(JointState.position[i]*10000)*jspn[i]
        s_meridim_js_sub[51+i*2]=round(JointState.position[11+i]*10000)*jspn[15+i]

##################################################################################################################################################
# dearpygui に よ る コ ン ソ ー ル 画 面 描 写 ##################################################################################################
##################################################################################################################################################

def main():
    
    global r_meridim
    global flag_ros1
    global jspn

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
        global error_count_esp_skip
        global error_count_pc_skip
        global start
        loop_count = 1
        error_count_pc_to_esp = 0
        error_count_esp_to_tsy = 0
        error_count_tsy_to_esp = 0
        error_count_esp_to_pc = 0
        error_count_tsy_skip = 0
        error_count_esp_skip = 0
        error_count_pc_skip = 0
        start = time.time() 
    
    while(True):
        
        # dpg描画 ==================================================
        dpg.create_context()
        dpg.create_viewport(title=TITLE_VERSION, width=600, height=520)

        # （画面左上）サーボ位置モニタリング用のウィンドウ ==================================================
        with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
            with dpg.group(label='LeftSide'): 
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[10,35+i*20], width=80)
            with dpg.group(label='RightSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[135,35+i*20], width=80)

        # （画面下段）メッセージ表示用ウィンドウ（アドレス・通信エラー等） ==================================================
        with dpg.window(label="Messege", width=590, height=155,pos=[5,360]):
            dpg.add_button(label="ResetCounter",  callback=reset_counter, width =90, pos=[470,30])
            dpg.add_text(message0,tag="DispMessage0")
            dpg.add_text(message1,tag="DispMessage1")
            dpg.add_text(message2,tag="DispMessage2")
            dpg.add_text(message3,tag="DispMessage3")
            dpg.add_text(message4,tag="DispMessage4")

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
            dpg.add_checkbox(label="Power", tag="Power",  callback=set_servo_power, pos=[8,50])

            dpg.add_checkbox(tag="Receive",  callback=set_resv_data, pos=[160,27])
            dpg.add_text("ESP32->", pos=[100,27])
            dpg.add_checkbox(tag="Send",  callback=set_send_data, pos=[160,50])
            dpg.add_text("ESP32<-", pos=[100,50])
            dpg.add_checkbox(tag="Virtual",  callback=set_send_virtual, pos=[160,73])
            dpg.add_text("Virtual<-", pos=[86,73])

            dpg.add_checkbox(label="->ROS1", tag="ROS1pub",  callback=ros1_pub, pos=[192,27])
            dpg.add_checkbox(label="<-ROS1", tag="ROS1sub",  callback=ros1_sub, pos=[192,50])
            dpg.add_checkbox(label="<-Demo", tag="Action",  callback=set_demo_action, pos=[192,73])

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
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            global r_meridim
            global s_meridim_motion

            #メッセージ欄の表示更新
            dpg.set_value("DispMessage0", message0) #メッセージ欄表示用
            dpg.set_value("DispMessage1", message1) #メッセージ欄表示用
            dpg.set_value("DispMessage2", message2) #メッセージ欄表示用
            dpg.set_value("DispMessage3", message3) #メッセージ欄表示用
            dpg.set_value("DispMessage4", message4) #メッセージ欄表示用

            #サーボデータとIMUデータの表示更新
            for i in range(0, 15, 1):
                #global button
                idld = r_meridim[21+i*2]
                idrd = r_meridim[51+i*2]
                idsensor = r_meridim[i+2]/10000
                dpg.set_value("ID L"+str(i), idld/100) #サーボIDと数値の表示L側
                dpg.set_value("ID R"+str(i), idrd/100) #サーボIDと数値の表示R側

                if i < 13: #IMUデータの更新
                    if i < 11:
                        dpg.set_value("mpu"+str(i),idsensor)
                    else:
                        dpg.set_value("mpu"+str(i),idsensor*100)

            #リモコンデータの表示更新
            dpg.set_value("pad_button", str(r_meridim[80]))
            dpg.set_value("pad_Lx", r_meridim_char[163])
            dpg.set_value("pad_Ly", r_meridim_char[162])
            dpg.set_value("pad_Rx", r_meridim_char[165])
            dpg.set_value("pad_Ry", r_meridim_char[164])
            padL2val = (r_meridim_char[167])
            if (padL2val<0):
                padL2val = 256+padL2val
            if (r_meridim[80]&256==0):
                padL2val = 0
            padR2val = (r_meridim_char[166])
            if (padR2val<0):
                padR2val = 256+padR2val
            if (r_meridim[80]&512==0):
                padR2val = 0
            dpg.set_value("pad_L2v", padL2val)
            dpg.set_value("pad_R2v", padR2val)
            dpg.set_value("button_data", r_meridim[80])

            # ROS1 joint_statesのパブリッシュ =====================================================================================
            if flag_ros1_pub:#ROS送信:joint_statesのpublishを実施
                if flag_ros1==0:
                    rospy.init_node('joint_state_meridim', anonymous=True)
                    flag_ros1=1
                joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
                rate = rospy.Rate(100) # 100hz
                js_meridim = JointState()
                js_meridim.header.stamp = rospy.Time.now()
                js_meridim.name =\
                    ['c_chest_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',\
                        'l_elbow_pitch',                 'l_hipjoint_yaw',                  'l_hipjoint_roll',                  'l_hipjoint_pitch',\
                            'l_knee_pitch',              'l_ankle_pitch',                   'l_ankle_roll',                     \
                                'c_head_yaw',            'r_shoulder_pitch',                'r_shoulder_roll',                  'r_elbow_yaw',\
                                    'r_elbow_pitch',     'r_hipjoint_yaw',                  'r_hipjoint_roll',                  'r_hipjoint_pitch',\
                                        'r_knee_pitch',  'r_ankle_pitch',                   'r_ankle_roll']
                js_meridim.position = \
                    [math.radians(s_meridim_motion[21]/100*jspn[0]), math.radians(s_meridim_motion[23]/100*jspn[1]), math.radians(s_meridim_motion[25]/100)*jspn[2], math.radians(s_meridim_motion[27]/100*jspn[3]),\
                        math.radians(s_meridim_motion[29]/100*jspn[4]), math.radians(s_meridim_motion[31]/100*jspn[5]), math.radians(s_meridim_motion[33]/100*jspn[6]), math.radians(s_meridim_motion[35]/100*jspn[7]), \
                            math.radians(s_meridim_motion[37]/100*jspn[8]),math.radians(s_meridim_motion[39]/100*jspn[9]), math.radians(s_meridim_motion[41]/100*jspn[10]),\
                                math.radians(s_meridim_motion[51]/100*jspn[15]),math.radians(s_meridim_motion[53]/100*jspn[16]), math.radians(s_meridim_motion[55]/100*jspn[17]), math.radians(s_meridim_motion[57]/100*jspn[18]), \
                                    math.radians(s_meridim_motion[59]/100*jspn[19]), math.radians(s_meridim_motion[61]/100*jspn[20]), math.radians(s_meridim_motion[63]/100*jspn[21]), math.radians(s_meridim_motion[65]/100*jspn[22]), \
                                        math.radians(s_meridim_motion[67]/100*jspn[23]),  math.radians(s_meridim_motion[69]/100*jspn[24]), math.radians(s_meridim_motion[71]/100*jspn[25])]

                js_meridim.velocity = []
                #js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                js_meridim.effort = []        
                joint_pub.publish(js_meridim)
                rate.sleep() 

            # R O S 1 joint_statesのサブスクライブ ====================================================================================
            if flag_ros1_sub:
                #joint_sub = rospy.Subscriber('joint_states', JointState, queue_size=10)
                if flag_ros1==0:
                    rospy.init_node('joint_state_meridim', anonymous=True)
                    flag_ros1=1      
                rospy.Subscriber('joint_states', JointState, callback)
                #rospy.spin()
                
            # ====================================================================================================

            #dpg表示更新処理
            dpg.render_dearpygui_frame()

        dpg.destroy_context()

#スレッド2つで送受信と画面描写を並列処理
if __name__ == '__main__':
    thread1 = threading.Thread(target=meridian_loop) #dearpyguiによるコンソール画面描写スレッド
    thread1.start()
    main()
