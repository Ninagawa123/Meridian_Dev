# !/usr/bin/python3
# coding: UTF-8
# もしくは　# !/usr/bin/env python など環境に合わせて

# Izumi Ninagawa & Meridian project
# MIT License

# 2023.12.30 フロー送信/ステップ送信の切り替え,データ表示のターゲット/実行結果の切り替え,自身のIPの自動取得
# 2023.12.30 これまでのグローバル変数をクラスに格納. 一部をnumpy化. sleepを入れてCPU負荷を軽減
# 2023.12.31 エラー受信値のオーバーフローバグを修正
# 2024.01.05 起動時のモードを"Actual"に修正
# 2024.01.05 ミニターミナルにset&sendを追加し, 送信方法を変更. 【Mini Terminal】参照
# 2024.01.07 ターミナルに送受信データを表示する機能を追加. 
# 2024.04.30 L2R2ボタンのアナログ値の表示を修正.
# 2025.01.26 genesis用のRedis入力を追加. POWERとRedisをチェックで動作.

# Meridian console 取扱説明書
#
# ・起動方法
# 当ファイルがあるディレクトリにて, ターミナルより
# python3 Meridian_console.py [送信先のESP32のIPアドレス　例:192.168.1.12]
# と入力して実行します. 必要に応じてライブラリをpip3で追加してください
# （IPアドレスなしで実行した場合は, 82行目のUDP_SEND_IP_DEFでの設定が反映されます）
# UDP_SEND_IPはESP32の起動時にPCシリアルモニタ上に表示されます
#
# ・各ウィンドウについて
# 【Axis Monitor】
# 各サーボの値です. パワーオン時にはスライダでサーボを動かすことができます
# Target/Actualのラジオボタンで, データの表示を送信データ/受信データに切り替えられます
#
# 【Sensor Monitor】
# MIUのデータを表示します. rol,pit,yawはセンサフュージョン値です. SetYawボタンでヨー軸の中央値をリセットできます
#
# 【Command】
# ◻︎ POWER : 全サーボのパワーをオンオフします
# ◻︎ DEMO  : サインカーブの全身モーションを計算します（<-ESP32オンで送信）
# ◻︎ Python: ユーザー定義のpythonコードを反映します
# ◻︎ Enable: Demoやpythonを送信に反映します
# ◻︎ ->ROS1: ROS1のjointデータをパブリッシュします（Rvisと連動できます）
# ◻︎ <-ROS1: ROS1のサブスクライブします（動作未確認）
# # Control Pad Monitor: リモコンの入力状態を標準化して表示します
#
# 【Mini Terminal】
# Meridim配列のデータをインプットし8つまで同時送信することができます
# MeridianのIndex（Meridim90であれば0~89）とDataを入力し, 
# Set&Sendボタンでデータを1回送信します. 
# Setを押した後、Continuousチェックを入れることで同じデータを毎フレーム送信します.  
# Indexの範囲外のデータは無効となり送信されません. また, チェックを外した時に送信バッファの各Indexに-1が代入されます
# ◯Flow ◯Step : フローモードとステップモードを切り替えます 
# [Next frame]: ステップモード時に1フレームだけ次に進みます
#
# 【Button Input】
# コンソールからリモコンボタン押下情報を送信します
#
# 【Message】
# IPと各経路のエラーカウント, エラー率, フレーム数, 動作周波数を表示します
# ResetCycle: 周波数のカウントをリセットするボタンです
# ResetCounter: カウンタの値をリセットするボタンです
# disp_send: 送信データをターミナルに表示するチェックボックスです
# disp_rcvd: 受信データをターミナルに表示するチェックボックスです
# TsySKIP, PcSKIP: 連番データの取りこぼし数を表示します
# PS4リモコン接続時に受信スキップ回数が5%ほど検出されるのは, 現在の仕様では正常な動作です

import sys
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
import redis  # Redis用ライブラリ

# ROS搭載マシンの場合はrospyをインポートする
try:
    import rospy
    rospy_imported = True
    from sensor_msgs.msg import JointState
except ImportError:
    rospy_imported = False
    print("rospy not found. ROS functions will be disabled.")


# 定数
TITLE_VERSION = "Meridian_Console_v25.0126" # DPGのウィンドウタイトル兼バージョン表示
UDP_RESV_PORT = 22222                       # 受信ポート
UDP_SEND_PORT = 22224                       # 送信ポート
UDP_SEND_IP_DEF = "192.168.1.xx"            # 送信先のESP32のIPアドレス 21
MSG_SIZE = 90                               # Meridim配列の長さ(デフォルトは90)
MSG_BUFF = MSG_SIZE * 2                     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2                     # Meridim配列のエラーフラグの格納場所（配列の最後から２番目）
MSG_CKSM = MSG_SIZE - 1                     # Meridim配列のチェックサムの格納場所（配列の末尾）
STEP = 94                                   # 1フレームあたりに増加させる制御処理用の数値,サインカーブを何分割するか

# Redisサーバー設定
REDIS_HOST = "localhost"
REDIS_PORT = 6379
REDIS_KEY = "meridis"

# マスターコマンド
MCMD_TORQUE_ALL_OFF = 0                     # すべてのサーボトルクをオフにする（脱力）
MCMD_UPDATE_YAW_CENTER = 10002              # センサの推定ヨー軸を現在値でゼロに
MCMD_ENTER_TRIM_MODE = 10003                # トリムモードに入る(現在不使用)
MCMD_CLEAR_SERVO_ERROR_ID = 10004           # 通信エラーのサーボのIDをクリア
MCMD_BOARD_TRANSMIT_ACTIVE = 10005          # ボードが定刻で送信を行うモード（デフォルト設定.PC側が受信待ち）
MCMD_BOARD_TRANSMIT_PASSIVE = 10006         # ボードが受信を待ち返信するモード（PC側が定刻送信）
MCMD_RESET_MRD_TIMER = 10007                # フレーム管理時計mrd_t_milを現在時刻にリセット

# ================================================================================================================
# ---- 変数の宣言 -------------------------------------------------------------------------------------------------
# ================================================================================================================
class MeridianConsole:
    def __init__(self):
        # Meridim配列関連
        self.r_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # Meridim配列
        self.s_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # Meridim配列
        self.r_meridim_char = np.zeros(MSG_SIZE*2, dtype=np.uint8)     # Meridim配列
        self.r_meridim_ushort = np.zeros(MSG_SIZE*2, dtype=np.uint8)   # Meridim配列
        self.d_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # 表示用
        self.s_meridim_js_sub_f = np.zeros(MSG_SIZE, dtype=float)      # ROSサブスクライブ用
        self.pad_button_panel_short = np.array([0], dtype=np.uint16)   # コンパネからのリモコン入力用
        self.s_meridim_motion_f = np.zeros(MSG_SIZE, dtype=float)      # PC側で作成したサーボ位置送信用
        self.s_meridim_motion_keep_f = np.zeros(MSG_SIZE, dtype=float)  # PC側で作成したサーボ位置キープ用
        self.s_minitermnal_keep = np.zeros((8, 2))  # コンパネからのリモコン入力用
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            self.s_minitermnal_keep[i][0] = -1

        # エラー集計表示用変数
        self.loop_count = 1              # フレーム数のカウンタ
        self.frame_sync_s = 0            # 送信するframe_sync_r(0-59999)
        self.frame_sync_r_expect = 0     # 毎フレームカウントし, 受信カウントと比較(0-59999)
        self.frame_sync_r_resv = 0       # 今回受信したframe_sync_r
        self.frame_sync_r_last = 0       # 前回受信したframe_sync_r
        self.error_count_esp_to_pc = 0   # PCからESP32へのUDP送信でのエラー数
        self.error_count_pc_to_esp = 0   # ESP32からPCへのUDP送信でのエラー数
        self.error_count_esp_to_tsy = 0  # ESPからTeensyへのSPI通信でのエラー数
        self.error_count_tsy_delay = 0   # Teensyのシステムディレイ
        self.error_count_tsy_to_esp = 0  # TeensyからESP32へのSPI通信でのエラー数
        self.error_count_tsy_skip = 0    # Teensyが受信したデータがクロックカウントスキップしていたか
        self.error_count_esp_skip = 0    # ESPが受信したデータがクロックカウントスキップしていたか
        self.error_count_pc_skip = 0     # PCが受信したデータがクロックカウントスキップしていたか
        self.error_count_servo_skip = 0  # マイコン(Teensy/ESP32)がサーボ値の受信に失敗した回数
        self.error_servo_id = "None"     # 受信エラーのあったサーボのIDを格納
        self.error_servo_id_past = 0     # 前回のサーボエラーIDキープ用

        # 制御コマンド用フラグ等
        self.command_send_trial = 1             # Commandを連続で送信する回数
        self.flag_update_yaw = 0                # IMUのヨー軸センターリセットフラグ(python内部用)
        self.flag_servo_power = 0               # 全サーボのパワーオンオフフラグ
        self.flag_udp_resv = True               # UDP受信の完了フラグ
        self.flag_enable_send_made_data = False # ESP32への状態データの送信のオンオフフラグ
        self.flag_resv_data = 0                 # ESP32からの状態データの受信のオンオフフラグ
        self.flag_send_data = 0                 # ESP32への状態データの送信のオンオフフラグ（サーボオフでも送信可）
        self.flag_send_virtual = 0              # ハードウェアを接続しないで動作させる場合のバーチャルハードのオンオフフラグ
        self.flag_send_motion = 0               # 計算モーション送信のオンオフフラグ
        self.flag_set_miniterminal_data = 0     # ミニターミナルの値をセットするボタンのためのフラグ
        self.flag_send_miniterminal_data_cont = 0    # ミニターミナルの値を送信するボタンのためのフラグ
        self.flag_send_miniterminal_data_once = 0    # ミニターミナルの値を1回送信する
        self.flag_tarminal_mode_send = 0        # miniterminalを有効にし, コマンドを優先する
        self.flag_demo_action = 0               # デモ/テスト用の計算モーション送信のオンオフフラグ
        self.flag_python_action = 0             # ユーザー自作python有効のオンオフフラグ
        self.flag_ros1 = 0                      # ROS1の起動init（初回のみ）
        self.flag_ros1_pub = 0                  # ROS1のjoint_statesのパブリッシュ
        self.flag_ros1_sub = 0                  # ROS1のjoint_statesのサブスクライブ
        self.flag_redis_sub = False             # Redisデータのサブスクライブ
        self.flag_set_flow_or_step = 1          # Meridianの循環を+:通常フロー, -:ステップ に切り替え
        self.flag_servo_home = 0                # 全サーボ位置をゼロリセット
        self.flag_stop_flow = False             # ステップモード中の待機フラグ
        self.flag_allow_flow = False            # ステップモード中に1回データを流すフラグ
        self.flag_display_mode = 0              # Axis monitorの表示 1:送信データ(target) 0:受信データ(actual)
        self.flag_ros1_output_mode = 0          # ROS1にパブリッシュするデータ 1:送信データ(target) 0:受信データ(actual)
        self.frag_reset_cycle = False           # ボードのフレームの周波数をリセットする
        self.frag_reset_errors = False          # Meridimのエラーカウントをリセットする
        self.flag_disp_send = 0                 # ターミナルに送信データを表示する
        self.flag_disp_rcvd = 0                 # ターミナルに受信データを表示する

        # メッセージ表示用
        self.message0 = "This PC's IP adress is "+get_local_ip()
        self.message1 = ""
        self.message2 = ""
        self.message3 = ""
        self.message4 = ""
        self.message3 = ""
        self.message4 = ""

        # モーション計算用変数
        self.x = 0  # 増分計算用 (STEPずつ)
        self.y = 0  # 増分計算用 (1ずつ)

        self.jspn = list(range(30))  # サーボ角度のROS joint_states変換用の回転方向順逆補正
        self.jspn[0] = 1   # 頭ヨー
        self.jspn[1] = 1   # 左肩ピッチ
        self.jspn[2] = 1   # 左肩ロール
        self.jspn[3] = 1   # 左肘ヨー
        self.jspn[4] = 1   # 左肘ピッチ
        self.jspn[5] = 1   # 左股ヨー
        self.jspn[6] = 1   # 左股ロール
        self.jspn[7] = 1   # 左股ピッチ
        self.jspn[8] = 1   # 左膝ピッチ
        self.jspn[9] = 1   # 左足首ピッチ
        self.jspn[10] = 1  # 左足首ロール
        self.jspn[11] = 1  # 予備
        self.jspn[12] = 1  # 予備
        self.jspn[13] = 1  # 予備
        self.jspn[14] = 1  # 予備
        self.jspn[15] = 1  # 腰ヨー
        self.jspn[16] = 1  # 右肩ピッチ
        self.jspn[17] = -1  # 右肩ロール
        self.jspn[18] = -1  # 右肘ヨー
        self.jspn[19] = 1  # 右肘ピッチ
        self.jspn[20] = -1  # 右股ヨー
        self.jspn[21] = -1  # 右股ロール
        self.jspn[22] = 1  # 右股ピッチ
        self.jspn[23] = 1  # 右膝ピッチ
        self.jspn[24] = 1  # 右足首ピッチ
        self.jspn[25] = -1  # 右足首ロール
        self.jspn[26] = 1  # 予備
        self.jspn[27] = 1  # 予備
        self.jspn[28] = 1  # 予備
        self.jspn[29] = 1  # 予備
        self.start = 0

        # ロックの追加
        self.lock = threading.Lock()

def get_local_ip(): # 自身のIPアドレスを自動取得する
    try:        
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # ダミーのsocketを作成
        s.connect(("8.8.8.8", 80)) # GoogleのDNSサーバーに接続を試み,実際には接続しない
        IP = s.getsockname()[0] # このsocketを通じて取得されるローカルIPアドレスを取得
        s.close()
        return IP
    except Exception as e:
        return "Error: " + str(e)

def get_udp_send_ip(): # コマンドライン引数が提供されているか確認
    if len(sys.argv) > 1: 
        return sys.argv[1]  # 最初の引数を返す
    else:
        return UDP_SEND_IP_DEF  # デフォルトのIPアドレス（またはエラーメッセージ）

UDP_SEND_IP = get_udp_send_ip()

# ================================================================================================================
# ---- メインループ ------------------------------------------------------------------------------------------------
# ================================================================================================================

# ------------------------------------------------------------------------
# [ 0 ]  初期設定
# ------------------------------------------------------------------------
mrd = MeridianConsole()  # Meridianデータのインスタンス

def redis_sub():
    """RedisのサブスクライブをON/OFF"""
    if mrd.flag_redis_sub:
        print("Stopping Redis subscription.")
        mrd.flag_redis_sub = False
    else:
        print("Starting Redis subscription.")
        mrd.flag_redis_sub = True

def fetch_redis_data():
    if not mrd.flag_redis_sub:
        return

    try:
        r = redis.StrictRedis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

        if not r.exists(REDIS_KEY):
            print("[Redis Error] Key 'meridis' not found.")
            return

        data = r.lrange(REDIS_KEY, 0, -1)
        print(f"[Debug] Raw data from Redis: {data}")  # <== 追加

        if len(data) != 90:
            print(f"[Redis Error] Expected 90 elements, but got {len(data)}.")
            return

        try:
            data = [float(x) for x in data]
            print(f"[Debug] Converted float data: {data}")  # <== 追加
        except ValueError:
            print("[Redis Error] Invalid data format. Could not convert to float.")
            return

        if mrd.flag_ros1_sub:
            print("[Debug] Skipping Redis data application due to ROS1 subscription.")
            return

        for i in range(21, 81, 2):
            mrd.s_meridim[i] = int(data[i] * 100)

        print(f"[Debug] Updated s_meridim with Redis data: {mrd.s_meridim[21:81:2]}")  # <== 追加

    except redis.ConnectionError:
        print("[Redis Error] Could not connect to Redis server.")
    except Exception as e:
        print(f"[Redis Error] Unexpected error: {str(e)}")


def meridian_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP用のsocket設定
    sock.bind((get_local_ip(), UDP_RESV_PORT))
    _checksum = np.array([0], dtype=np.int16)
    atexit.register(cleanup)  # この行は機能しているかどうかわからない

    while (True):
        print("Start.")
        _r_bin_data_past = np.zeros(180, dtype=np.int8) # 180個の要素を持つint8型のNumPy配列を作成
        _r_bin_data = np.zeros(180, dtype=np.int8)
        mrd.message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."

        with closing(sock):
            while True:
                mrd.loop_count += 1  # このpythonを起動してからのフレーム数をカウントアップ
                _r_bin_data_past = _r_bin_data
                _r_bin_data, addr = sock.recvfrom(MSG_BUFF)  # UDPに受信したデータを転記

# ------------------------------------------------------------------------
# [ 1 ] : UDPデータの受信
# ------------------------------------------------------------------------
# [ 1-1 ] : UDPデータの受信を待つループ
                while np.array_equal(_r_bin_data_past, _r_bin_data): #前回受信データと差分があったら進む
                    _r_bin_data, addr = sock.recvfrom(MSG_BUFF)

# [ 1-2 ] : 受信UDPデータの変換
                mrd.r_meridim = struct.unpack('90h', _r_bin_data) # 受信データをshort型のMeridim90に変換
                mrd.r_meridim_ushort = struct.unpack(
                    '90H', _r_bin_data)  # unsignedshort型
                mrd.r_meridim_char = struct.unpack('180b', _r_bin_data)
                mrd.message1 = "UDP data receiving from "+UDP_SEND_IP  # 受信中のメッセージ表示

# [ 1-3 ] : 送信UDPデータのターミナル表示
                if mrd.flag_disp_send:
                    mrd.flag_disp_send=1
                    print('send:'+' '.join(map(str, mrd.s_meridim)))
                    
# [ 1-4 ] : 受信UDPデータのターミナル表示
                if mrd.flag_disp_rcvd:
                    mrd.flag_disp_rcvd=1
                    print('rcvd:'+' '.join(map(str, mrd.r_meridim)))

# ------------------------------------------------------------------------
# [ 2 ] : 受信データのチェック
# ------------------------------------------------------------------------
# [ 2-1 ] : チェックサムの確認
                _checksum[0] = np.int16(~np.sum(mrd.r_meridim[:MSG_SIZE-1], dtype=np.int16)) # 受信データに対するチェックサム値の計算
                _temp_int16 = np.int16(0)  # エラーフラグのカウント用変数

                if _checksum[0] != mrd.r_meridim[MSG_SIZE-1]:  # チェックサムがNGの処理
                    # エラーフラグ15ビット目:PCのUDP受信エラーフラグを上げる
                    _temp_int16 = (mrd.r_meridim[MSG_ERRS] & 0xFFFF) | 0b1000000000000000
                    mrd.error_count_esp_to_pc += 1  # PCのUDP受信エラーをカウントアップ
# [ 2-2 ] : チェックサムOKデータのエラーフラグ処理
                else:
                    # エラーフラグを調べる
                    if (mrd.r_meridim[MSG_ERRS] >> 14 & 1) == 1: # 14ビット目:ESP32のPCからのUDP受信のエラー
                        mrd.error_count_pc_to_esp += 1
                    if (mrd.r_meridim[MSG_ERRS] >> 13 & 1) == 1: # 13ビット目:TeensyのESP32からのSPI受信のエラー
                        mrd.error_count_esp_to_tsy += 1
                    if (mrd.r_meridim[MSG_ERRS] >> 12 & 1) == 1: # 12ビット目:ESPのTeensyからのSPI受信のエラー
                        mrd.error_count_tsy_to_esp += 1
                    if (mrd.r_meridim[MSG_ERRS] >> 11 & 1) == 1: # 11ビット目:Teensyのシステムディレイのエラー
                        mrd.error_count_tsy_delay += 1
                    if (mrd.r_meridim[MSG_ERRS] >> 10 & 1) == 1: # 10ビット目:ESPのPCからのUDP受信のフレーム連番スキップフラグ
                        mrd.error_count_esp_skip += 1                    
                    if (mrd.r_meridim[MSG_ERRS] >> 9 & 1) == 1:  # 9ビット目:TeensyのESP経由のPCから受信のフレーム連番スキップフラグ
                        mrd.error_count_tsy_skip += 1
                    _temp_int16 = mrd.r_meridim[MSG_ERRS] & 0b0000000011111111  # サーボ値の受信に失敗したサーボID(エラーフラグ下位8ビット）を調べる
                    mrd.error_servo_id_past = mrd.error_servo_id
                    if _temp_int16 > 0:
                        mrd.error_count_servo_skip += 1
                        if mrd.r_meridim[MSG_ERRS] & 0b0000000011111111 > 99:
                            mrd.error_servo_id = "id_R" + str(int(mrd.r_meridim[MSG_ERRS] & 0b0000000011111111)-100)
                        else:
                            mrd.error_servo_id = "id_L" + str(int(mrd.r_meridim[MSG_ERRS] & 0b0000000011111111))
                    else:
                        mrd.error_servo_id = "None"
                        
# [ 2-3 ] : 断線エラーのサーボID表示処理
                    if (mrd.error_servo_id_past != mrd.error_servo_id):
                        if mrd.error_servo_id == "None":
                            print("Servo error gone...")
                        else:
                            print("Found servo error: "+mrd.error_servo_id)

# [ 2-4 ] : 末端処理
                    _temp_int16 = mrd.r_meridim[MSG_ERRS] & 0b0111111111111111 # エラーフラグ15ビット目(PCのUDP受信エラーフラグ)を下げる
                    mrd.frame_sync_r_resv = mrd.r_meridim_ushort[1] # フレームスキップチェック用のカウントの代入

# ------------------------------------------------------------------------
# [ 3 ] : ステップモードの場合はここでループ待機
# ------------------------------------------------------------------------
                if mrd.flag_set_flow_or_step < 0:
                    while mrd.flag_stop_flow:
                        if mrd.flag_allow_flow:
                            print("break")
                            break
                        time.sleep(0.005)  # CPUの負荷を下げる
                    time.sleep(0.001)
                    mrd.flag_stop_flow = True

# ------------------------------------------------------------------------
# [ 4 ] : チェックOKの受信UDPデータについての処理
# ------------------------------------------------------------------------
# [ 4-1 ] : チェックサムがOK かつ シーケンス番号が前回と異なっていれば, 処理に回す
                if (_checksum[0] == mrd.r_meridim[MSG_SIZE-1]) and (mrd.frame_sync_r_resv != mrd.frame_sync_r_last):

                    # 受信データを送信データに転記
                    for i in range(MSG_SIZE-1):
                        mrd.s_meridim[i] = mrd.r_meridim[i]

# [ 4-2 ] : シーケンス番号の処理
                    mrd.frame_sync_r_expect += 1  # 予想シーケンス番号のカウントアップ
                    if mrd.frame_sync_r_expect > 59999:
                        mrd.frame_sync_r_expect = 0

                    if (mrd.frame_sync_r_resv == mrd.frame_sync_r_expect): # 受信したカウントが予想通りであればスキップなし
                        _temp_int16 &= 0b1111111011111111 # PCのESP経由Teensyからの連番スキップフラグを下げる
                    else:
                        _temp_int16 |= 0b0000000100000000 # PCのESP経由Teensyからの連番スキップフラグを上げる
                        mrd.frame_sync_r_expect = mrd.frame_sync_r_resv # 受信カウントの方が多ければズレを検出し, 追いつく
                        mrd.error_count_pc_skip += 1  # スキップカウントをプラス

# [ 4-3 ] : 最終サーボ位置情報のキープ
                    if mrd.flag_servo_power == 2: # サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                        for i in range(21, 81, 2):
                            mrd.s_meridim_motion_keep_f[i] = mrd.r_meridim[i]*0.01
                        mrd.flag_servo_power = 1

                    if mrd.flag_servo_power == -1: # サーボオフボタン押下初回のみ最終送信サーボ情報をキープ
                        for i in range(21, 81, 2):
                            mrd.s_meridim_motion_keep_f[i] = mrd.s_meridim[i]*0.01
                        mrd.flag_servo_power = 0

# ------------------------------------------------------------------------
# [ 5 ] : 送信用UDPデータの作成
# ------------------------------------------------------------------------
# [ 5-1 ] : 送信するサーボ位置を以下の場合別に s_meridim_motion に格納
                    if _checksum[0] == mrd.r_meridim[MSG_SIZE-1]:  # 受信成功時はデータ更新
                        mrd.s_meridim.fill(0)  # 配列内のデータをゼロでリセット

# ▶︎ 5-1-1 : ① 受信値そのままの場合：送信データのベースを受信データのコピーで作成
                    if mrd.flag_servo_power:  # サーボパワーオン時は, 電源入力時に保持した値を固定で流す（ハウリング的なサーボ位置ズレの増幅を防止）
                        for i in range(21, 81, 2):
                            mrd.s_meridim[i] = int(
                                mrd.s_meridim_motion_keep_f[i]*100)
                    else:
                        if mrd.flag_resv_data:
                            for i in range(21, 81, 2):  # 受信サーボ値を書き込みモーションのベースとして一旦キープ
                                mrd.s_meridim_motion_f[i] = mrd.r_meridim[i]*0.01

# ▶︎ 5-1-2 : ② サーボ位置にROSのサブスクライブを反映させる場合にはここでデータを作成★★
                    if mrd.flag_ros1_sub:
                        for i in range(15):
                            mrd.s_meridim_motion_f[21+i *
                                                   2] = mrd.s_meridim_js_sub_f[21+i*2]
                            mrd.s_meridim_motion_f[51+i *
                                                   2] = mrd.s_meridim_js_sub_f[51+i*2]

# ▶︎ 5-1-3 : ③ サーボ位置をここで計算制御する場合は以下でデータを作成(まずはデモモーションのみで運用テスト)
                    if mrd.flag_demo_action:
                        # xをフレームごとにカウントアップ
                        mrd.x += math.pi/STEP
                        if mrd.x > math.pi*2000:
                            mrd.x = 0
                        # サインカーブで全身をくねらせる様にダンス
                        mrd.s_meridim_motion_f[21] = int(np.sin(mrd.x)*30)           # 頭ヨー
                        mrd.s_meridim_motion_f[23] = int(np.sin(mrd.x)*10) + 20    # 左肩ピッチ
                        mrd.s_meridim_motion_f[25] = -int(np.sin(mrd.x*2)*10) + 10  # 左肩ロール
                        mrd.s_meridim_motion_f[27] = int(np.sin(mrd.x)*10) + 10    # 左肘ヨー
                        mrd.s_meridim_motion_f[29] = int(np.sin(mrd.x)*30) - 30    # 左肘ピッチ
                        mrd.s_meridim_motion_f[31] = int(np.sin(mrd.x)*5)            # 左股ヨー
                        mrd.s_meridim_motion_f[33] = -int(np.sin(mrd.x)*4)           # 左股ロール
                        mrd.s_meridim_motion_f[35] = int(np.sin(mrd.x*2)*20) - 2   # 左股ピッチ
                        mrd.s_meridim_motion_f[37] = -int(np.sin(mrd.x*2)*40)        # 左膝ピッチ
                        mrd.s_meridim_motion_f[39] = int(np.sin(mrd.x*2)*20) - 2   # 左足首ピッチ
                        mrd.s_meridim_motion_f[41] = int(np.sin(mrd.x)*4)            # 左足首ロール
                        mrd.s_meridim_motion_f[51] = -int(np.sin(mrd.x)*20)          # 腰ヨー
                        mrd.s_meridim_motion_f[53] = -int(np.sin(mrd.x)*10) + 20   # 右肩ピッチ
                        mrd.s_meridim_motion_f[55] = -int(np.sin(mrd.x*2)*10) + 10  # 右肩ロール
                        mrd.s_meridim_motion_f[57] = -int(np.sin(mrd.x)*10) + 10   # 右肘ヨー
                        mrd.s_meridim_motion_f[59] = -int(np.sin(mrd.x)*30) - 30   # 右肘ピッチ
                        mrd.s_meridim_motion_f[61] = -int(np.sin(mrd.x)*5)           # 右股ヨー
                        mrd.s_meridim_motion_f[63] = int(np.sin(mrd.x)*4)            # 右股ロール
                        mrd.s_meridim_motion_f[65] = -int(np.sin(mrd.x*2)*20) - 2  # 右股ピッチ
                        mrd.s_meridim_motion_f[67] = int(np.sin(mrd.x*2)*40)         # 右膝ピッチ
                        mrd.s_meridim_motion_f[69] = -int(np.sin(mrd.x*2)*20) - 2  # 右足首ピッチ
                        mrd.s_meridim_motion_f[71] = -int(np.sin(mrd.x)*4)           # 右足首ロール
                        if mrd.flag_enable_send_made_data:
                            for i in range(15):
                                mrd.s_meridim_motion_keep_f[21+i * 2] = mrd.s_meridim_motion_f[21+i*2]
                                mrd.s_meridim_motion_keep_f[51+i * 2] = mrd.s_meridim_motion_f[51+i*2]

# ▶︎ 5-1-4 : ④ ユーザーがサーボ位置処理を反映させる場合 → ここで自由にコードを作成
                    # redisからのデータを仮にここで処理
                    fetch_redis_data()
        
                    if mrd.flag_python_action:  # コード書式は自由だが, 仮にすべての関節角度に0を代入する場合の例
                        mrd.s_meridim_motion_f[21] = mrd.s_meridim_motion_f[21] # 頭ヨー
                        mrd.s_meridim_motion_f[23] = mrd.s_meridim_motion_f[23] # 左肩ピッチ
                        mrd.s_meridim_motion_f[25] = mrd.s_meridim_motion_f[25] # 左肩ロール
                        mrd.s_meridim_motion_f[27] = mrd.s_meridim_motion_f[27] # 左肘ヨー
                        mrd.s_meridim_motion_f[29] = mrd.s_meridim_motion_f[29] # 左肘ピッチ
                        mrd.s_meridim_motion_f[31] = mrd.s_meridim_motion_f[31] # 左股ヨー
                        mrd.s_meridim_motion_f[33] = mrd.s_meridim_motion_f[33] # 左股ロール
                        mrd.s_meridim_motion_f[35] = mrd.s_meridim_motion_f[35] # 左股ピッチ
                        mrd.s_meridim_motion_f[37] = mrd.s_meridim_motion_f[37] # 左膝ピッチ
                        mrd.s_meridim_motion_f[39] = mrd.s_meridim_motion_f[39] # 左足首ピッチ
                        mrd.s_meridim_motion_f[41] = mrd.s_meridim_motion_f[41] # 左足首ロール
                        mrd.s_meridim_motion_f[51] = mrd.s_meridim_motion_f[51] # 腰ヨー
                        mrd.s_meridim_motion_f[53] = mrd.s_meridim_motion_f[53] # 右肩ピッチ
                        mrd.s_meridim_motion_f[55] = mrd.s_meridim_motion_f[55] # 右肩ロール
                        mrd.s_meridim_motion_f[57] = mrd.s_meridim_motion_f[57] # 右肘ヨー
                        mrd.s_meridim_motion_f[59] = mrd.s_meridim_motion_f[59] # 右肘ピッチ
                        mrd.s_meridim_motion_f[61] = mrd.s_meridim_motion_f[61] # 右股ヨー
                        mrd.s_meridim_motion_f[63] = mrd.s_meridim_motion_f[63] # 右股ロール
                        mrd.s_meridim_motion_f[65] = mrd.s_meridim_motion_f[65] # 右股ピッチ
                        mrd.s_meridim_motion_f[67] = mrd.s_meridim_motion_f[67] # 右膝ピッチ
                        mrd.s_meridim_motion_f[69] = mrd.s_meridim_motion_f[69] # 右足首ピッチ
                        mrd.s_meridim_motion_f[71] = mrd.s_meridim_motion_f[71] # 右足首ロール

# [ 5-2 ] : サーボ位置リセットボタン(Home)が押下されていたら全サーボ位置をゼロリセット
                    if mrd.flag_servo_home > 0:
                        for i in range(15):
                            mrd.s_meridim[21+i*2] = 0
                            mrd.s_meridim[51+i*2] = 0
                            mrd.s_meridim_motion_f[21+i*2] = 0
                            mrd.s_meridim_motion_f[51+i*2] = 0
                            mrd.s_meridim_motion_keep_f[21+i*2] = 0
                            mrd.s_meridim_motion_keep_f[51+i*2] = 0                            
                        mrd.flag_servo_home = 0
                        
# [ 5-3 ] : PC側発行のサーボ位置をs_meridimに書き込む
                    if mrd.flag_enable_send_made_data: #PC側発行データの送信Enable判定
                        for i in range(21, 81, 2):
                            if mrd.flag_demo_action | mrd.flag_python_action | mrd.flag_ros1_sub:
                                mrd.s_meridim[i] = int(mrd.s_meridim_motion_f[i]*100)
                            else:  # Consoleでモーションを指定しない場合はハンチング防止としてサーボオフ時のデータを送信
                                mrd.s_meridim[i] = int(mrd.s_meridim_motion_keep_f[i]*100)
                                
# [ 5-4 ] : サーボオンオフフラグチェック：サーボオンフラグを格納
                    if mrd.flag_servo_power > 0:
                        for i in range(20, 80, 2):
                            mrd.s_meridim[i] = 1
                    else:
                        for i in range(20, 80, 2):
                            mrd.s_meridim[i] = 0
                 
# [ 5-5 ] : リモコンデータをリセットし, PCからのリモコン入力値を格納
                    temp = np.array([0], dtype=np.int16)
                    temp[0] = 0
                    temp[0] = mrd.pad_button_panel_short[0]  # ボタンのショート型変換
                    mrd.s_meridim[15] = temp[0]  # ボタン
                    mrd.s_meridim[16] = 0  # アナログ1
                    mrd.s_meridim[17] = 0  # アナログ2
                    mrd.s_meridim[18] = 0  # アナログ3
                            
# [ 5-6 ] : 送信マスターコマンドの作成
                    mrd.s_meridim[0] = MSG_SIZE  # デフォルト値を格納

# ▶︎ 5-6-1 : ヨー軸センターリセットコマンドを格納
                    if (mrd.flag_update_yaw > 0):
                        mrd.flag_update_yaw -= 1
                        mrd.s_meridim[0] = MCMD_UPDATE_YAW_CENTER
                        if (mrd.flag_update_yaw == 0):
                            print(
                                "Send COMMAND 'Set Yaw Center.':["+str(MCMD_UPDATE_YAW_CENTER)+"]")

# ▶︎ 5-6-2 : フローモード(ボード側が周期制御を持つ)への切り替え
                    if mrd.flag_set_flow_or_step == 2:  
                        mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_ACTIVE
                        mrd.flag_set_flow_or_step = 1

# ▶︎ 5-6-3 : ステップモード(PC側が周期制御を持つ)への切り替え
                    if mrd.flag_set_flow_or_step == -2: 
                        mrd.s_meridim[0] = MCMD_BOARD_TRANSMIT_PASSIVE
                        mrd.flag_set_flow_or_step = -1

# ▶︎ 5-6-4 : ボード側のフレーム管理時計を現在時間でリセット
                    if mrd.frag_reset_cycle: 
                        mrd.s_meridim[0] = MCMD_RESET_MRD_TIMER
                        mrd.frag_reset_cycle = False

# [ 5-7 ] : エラーフラグの処理
# ▶︎ 5-7-1 : Meridim内のエラーフラグをリセット
                    if mrd.frag_reset_errors: 
                        mrd.s_meridim[MSG_ERRS] = 0
                        mrd.error_servo_id = "None"     # 受信エラーのあったサーボのIDを格納
                        mrd.error_servo_id_past = "None"     # 受信エラーのあったサーボのIDを格納
                        mrd.frag_reset_errors = False

# ▶︎ 5-7-2 : キープしたエラーフラグを格納
                    mrd.s_meridim[MSG_ERRS] = _temp_int16

# [ 5-8 ] : 送信用シーケンス番号の作成と格納
                    mrd.frame_sync_s += 1  # 送信用のframe_sync_sをカウントアップ
                    if mrd.frame_sync_s > 59999:  # 60,000以上ならゼロリセット
                        mrd.frame_sync_s = 0
                    if mrd.frame_sync_s > 32767:  # unsigned short として取り出せるようなsinged shortに変換
                        mrd.s_meridim[1] = mrd.frame_sync_s-65536
                    else:
                        mrd.s_meridim[1] = mrd.frame_sync_s  # & 0xffff

# [ 5-9 ] : ミニターミナルからのデータ送信処理
                    if mrd.flag_tarminal_mode_send > 0:  # ミニターミナルの送信モードの確認
                        print_string = ""
                        for i in range(8):
                            if ((mrd.s_minitermnal_keep[i][0] >= 0) and (mrd.s_minitermnal_keep[i][0] < MSG_SIZE)):
                                mrd.s_meridim[int(mrd.s_minitermnal_keep[i][0])] = int(mrd.s_minitermnal_keep[i][1])
                                print_string = print_string + "["+str(int(mrd.s_minitermnal_keep[i][0]))+"] " + str(int(mrd.s_minitermnal_keep[i][1]))+", "
                                # サーボパワーオン時のキープ配列にも反映しておくこうするとミニターミナルから脱力してサーボを回転させた後にサーボパワーオンで位置の固定ができる
                                mrd.s_meridim_motion_keep_f[int(mrd.s_minitermnal_keep[i][0])] = int(
                                    mrd.s_minitermnal_keep[i][1]*0.01)

                        if mrd.flag_tarminal_mode_send == 2:  # 送信データを一回表示
                            print("Sending data : ")
                            print(print_string[:-2])  # 末尾のカンマ以外を表示
                            mrd.flag_tarminal_mode_send = 1
                            
                        if mrd.flag_send_miniterminal_data_once == 1:    # ミニターミナルの値を1回送信する
                            mrd.flag_tarminal_mode_send = 0
                            mrd.flag_send_miniterminal_data_once = 0


# [ 5-10 ] : 格納した送信データについてチェックサムを追加
                    s_meridim_int16 = np.array(mrd.s_meridim[:MSG_SIZE-1], dtype=np.int16)
                    _checksum[0] = np.int16(~np.sum(s_meridim_int16, dtype=np.int16))
                    mrd.s_meridim[MSG_SIZE-1] = _checksum[0]

# ------------------------------------------------------------------------
# [ 6 ] : UDPデータを送信
# ------------------------------------------------------------------------
                    s_bin_data = struct.pack('90h', *mrd.s_meridim)        # データをパック
                    sock.sendto(s_bin_data, (UDP_SEND_IP, UDP_SEND_PORT))  # UDP送信
                    now = time.time()-mrd.start+0.0001

# ------------------------------------------------------------------------
# [ 7 ] : 表示処理
# ------------------------------------------------------------------------
# [ 7-1 ] : Axis monitor の表示データ切り替え
                    if mrd.flag_display_mode: # 1=target data(send data),0= actual data(received data)
                        # 送信データを表示用データに転記
                        for i in range(MSG_SIZE-1):
                            mrd.d_meridim[i] = mrd.s_meridim[i]
                    else:
                        # 受信データを表示用データに転記
                        for i in range(MSG_SIZE-1):
                            mrd.d_meridim[i] = mrd.r_meridim[i]

# [ 7-2 ] : メッセージウィンドウの表示更新
                    mrd.message2 = "ERROR COUNT ESP-PC:"+str("{:}".format(mrd.error_count_esp_to_pc)) + " PC-ESP:"+str("{:}".format(mrd.error_count_pc_to_esp))+" ESP-TSY:"+str(
                        "{:}".format(mrd.error_count_esp_to_tsy)) + " TSY_Delay:"+str("{:}".format(mrd.error_count_tsy_delay)) + "    Servo_trouble:"+mrd.error_servo_id

                    mrd.message3 = "ERROR RATE ESP-PC:"+str("{:.2%}".format(mrd.error_count_esp_to_pc/mrd.loop_count)) + " PC-ESP:"+str("{:.2%}".format(mrd.error_count_pc_to_esp/mrd.loop_count))+" ESP-TSY:"+str("{:.2%}".format(
                        mrd.error_count_esp_to_tsy/mrd.loop_count)) + " TsySKIP:"+str("{:.2%}".format(mrd.error_count_tsy_skip/mrd.loop_count)) + " ESPSKIP:" + str("{:.2%}".format(mrd.error_count_esp_skip/mrd.loop_count))

                    mrd.message4 = "SKIP COUNT Tsy:" + str("{:}".format(mrd.error_count_tsy_skip))+" ESP:"+str("{:}".format(mrd.error_count_esp_skip))+" PC:"+str("{:}".format(mrd.error_count_pc_skip)) + " Servo:"+str(
                        "{:}".format(mrd.error_count_servo_skip))+" PCframe:"+str(mrd.loop_count)+" BOARDframe:"+str(mrd.frame_sync_r_resv)+" "+str(int(mrd.loop_count/now))+"Hz"

                    # 今回受信のシーケンス番号を次回比較用にキープ
                    mrd.frame_sync_r_last = mrd.frame_sync_r_resv

# ------------------------------------------------------------------------
# [ 8 ] : シーケンス番号が更新されていなければ待機して[1-1]]に戻る
# ------------------------------------------------------------------------
                else:
                    time.sleep(0.001)


# ================================================================================================================
# ---- 関 数 各 種 ---------------------------------------------------------------------------------------------------
# ================================================================================================================

# ctrl+cで終了したときにも確実にソケットを閉じる試み（いまのところ機能していないかも）
def cleanup():  
    print("Meridan_console quited.")

# フラグ切り替え用の真偽反転
def flip_number(appdata, string1, string2):  
    if appdata == True:
        print(string1)
        return True
    else:
        print(string2)
        return False

# 押しボタンでフラグを立てる
def push_button_flag(string):  
    print(string)
    return 1

# [Axis Monitor] ウィンドウのスライダー処理
def set_servo_angle(channel, app_data):
    if channel[3] == "L":
        mrd.s_meridim[int(channel[4:6])*2+21] = int(app_data * 100)
        mrd.s_meridim_motion_f[int(channel[4:6])*2+21] = app_data
        print(f"L{channel[4:6]}[{int(channel[4:6])*2+21}]:{int(app_data*100)}")
    if channel[3] == "R":
        mrd.s_meridim[int(channel[4:6])*2+51] = int(app_data * 100)
        mrd.s_meridim_motion_f[int(channel[4:6])*2+51] = app_data
        print(f"R{channel[4:6]}[{int(channel[4:6])*2+51}]:{int(app_data*100)}")
    
# [Axis Monitor] ウィンドウのTarget, Actual 切り替えラジオボタン処理
def change_display_mode(sender, app_data, user_data):
    chosen_option = dpg.get_value(sender)
    if chosen_option == "Target":
        mrd.flag_display_mode = 1
        print("Target mode selected")
    elif chosen_option == "Actual":
        mrd.flag_display_mode = 0
        print("Actual mode selected")

# [Axis Monitor] ウィンドウのhomeボタン処理
def set_servo_home():
    mrd.flag_servo_home = push_button_flag("Set all servo position zero.")

# [Message] ウィンドウの送信データ表示処理
def set_disp_send():
    if mrd.flag_disp_send == 0:
        mrd.flag_disp_send = 2
        print("Display send meridim data.")
    else:
        mrd.flag_disp_send = 0
        print("Stop to display send meridim data.")

# [Message] ウィンドウの受信データ表示処理
def set_disp_rcvd():
    if mrd.flag_disp_rcvd == 0:
        mrd.flag_disp_rcvd = 2
        print("Display received meridim data.")
    else:
        mrd.flag_disp_rcvd = 0
        print("Stop to display received meridim data.")

# [Message] ウィンドウの reset cycle ボタン処理
def reset_cycle():  # カウンターのリセット
    mrd.frag_reset_cycle = True
    
# [Message] ウィンドウの reset counter ボタン処理
def reset_counter():  # カウンターのリセット
    mrd.loop_count = 1
    mrd.error_count_pc_to_esp = 0
    mrd.error_count_esp_to_tsy = 0
    mrd.error_count_tsy_to_esp = 0
    mrd.error_count_esp_to_pc = 0
    mrd.error_count_tsy_skip = 0
    mrd.error_count_esp_skip = 0
    mrd.error_count_pc_skip = 0
    mrd.error_count_servo_skip = 0
    mrd.frag_reset_errors = True
    mrd.error_servo_id = "None"
    mrd.start = time.time()

# [Button Input] ウィンドウ のリモコンボタン処理
def pad_btn_panel_on(sender, app_data, user_data):
    mrd.pad_button_panel_short
    if (mrd.pad_button_panel_short[0] & user_data) == 0:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] | user_data
        print(f'Btn:{mrd.pad_button_panel_short[0]}')
    else:
        mrd.pad_button_panel_short[0] = mrd.pad_button_panel_short[0] ^ user_data
        print(f'Btn:{mrd.pad_button_panel_short[0]}')

# [sensor monitor] ウィンドウのSetYawボタン処理
def set_yaw_center():  # IMUのヨー軸センターリセットフラグをcommand_send_trial回上げる（コマンドをcommand_send_trial回送信する）
    mrd.flag_update_yaw = mrd.command_send_trial
        
# [command] ウィンドウのPowerフラグ処理（サーボのオンオフ）
def set_servo_power(sender, app_data, user_data):  
    if app_data:
        mrd.flag_servo_power = 2
        print("Servo Power ON")
    else:
        mrd.flag_servo_power = -1
        print("Servo Power OFF")

# [command] ウィンドウのDemoフラグ処理
def set_demo_action(sender, app_data, user_data):  # チェックボックスに従いアクション送信フラグをオンオフ
    #    mrd.flag_demo_action=flip_number(mrd.flag_demo_action,"Start DEMO motion data streaming.","Quit DEMO motion data streaming.")
    mrd.flag_demo_action = flip_number(app_data, "Start DEMO motion data streaming.", "Quit DEMO motion data streaming.")

# [command] ウィンドウのPythonフラグ処理
def set_python_action(sender, app_data, user_data):  # チェックボックスに従いアクション送信フラグをオンオフ
    #    mrd.flag_python_action=flip_number(mrd.flag_python_action,"Start python motion data streaming.","Quit python motion data streaming.")
    mrd.flag_python_action = flip_number(app_data, "Start python motion data streaming.", "Quit python motion data streaming.")

# [command] ウィンドウのEnableフラグ処理
def set_enable(sender, app_data, user_data):  # チェックボックスに従いデータ送信フラグをオンオフ
    mrd.flag_enable_send_made_data = flip_number(app_data, "Start sending data to ESP32.", "Quit sending data to ESP32.")

# [command] ウィンドウのROS1データ送信モードをtarget/actualに切り替え
def change_ros1_output_mode(sender, app_data, user_data):
    mrd.flag_ros1_output_mode = flip_number(app_data, "Set target data(send data) as ROS1 publish.", "Set actual data(received data) as ROS1 publish.")

# [command] ウィンドウのROS1パブリッシュをオンオフ
def ros1_pub():
    if mrd.flag_ros1_pub == 0:
        mrd.flag_ros1_pub = 1
        print("Start publishing ROS1 joint_states.")
    else:
        mrd.flag_ros1_pub = 0
        print("Quit publishing ROS1 joint_states.")

# [command] ウィンドウのROS1サブスクライブをオンオフ
def ros1_sub():
    if mrd.flag_ros1_sub == 0:
        mrd.flag_ros1_sub = 1
        print("Start subscribing ROS1 joint_states.")
    else:
        mrd.flag_ros1_sub = 0
        print("Quit publishing ROS1 joint_states.")

# [command] ウィンドウのROS1データ変換
def joinstate_to_meridim(JointState):
    for i in range(11):
        mrd.s_meridim_js_sub_f[21+i * 2] = round(math.degrees(JointState.position[i])*100)*mrd.jspn[i]
        mrd.s_meridim_js_sub_f[51+i * 2] = round(math.degrees(JointState.position[11+i])*100)*mrd.jspn[15+i]

# [Mini Terminal] ウィンドウのsetボタン処理
def set_miniterminal_data():  # ミニターミナルのセットボタンが押下されたら送信データをセットする
    print_string = ""
    for i in range(8):
        _value_tag_index = "s_index"+str(i)
        _value_tag_data = "s_data"+str(i)

        if dpg.get_value(_value_tag_index) != "":
            if dpg.get_value(_value_tag_data) != "":
                if (int(dpg.get_value(_value_tag_data)) >= -32768) and (int(dpg.get_value(_value_tag_data)) <= 32767) and (int(dpg.get_value(_value_tag_index)) >= 0) and (int(dpg.get_value(_value_tag_index)) < MSG_SIZE):
                    print_string = print_string + "[" + str(dpg.get_value(_value_tag_index)) + "] " + str(dpg.get_value(_value_tag_data)) + ", "
                    mrd.s_minitermnal_keep[i][0] = int(dpg.get_value(_value_tag_index))
                    mrd.s_minitermnal_keep[i][1] = int(dpg.get_value(_value_tag_data))
                else:
                    # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
                    mrd.s_minitermnal_keep[i][0] = -1
                    print_string = print_string + \
                        "["+str(dpg.get_value(_value_tag_index)) + \
                        "] out of range, "
    print("Set mini tarminal data : ")
    print(print_string[:-2])  # 末尾のカンマ以外を表示
    

# [Mini Terminal] ウィンドウのSendボタン処理
def set_tarminal_continuous_on(sender, app_data):  # ボタン押下でset_flowフラグをオン
    if app_data:
        mrd.flag_tarminal_mode_send = 2
        print("Start to send miniterminal data.")
    else:
        mrd.flag_tarminal_mode_send = 0
        print("Stop to send miniterminal data.")
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][0] = -1
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][1] = 0
            
# [Mini Terminal] ウィンドウのSendボタン処理


def set_tarminal_send_on():  # ボタン押下でset_flowフラグをオン
    if mrd.flag_tarminal_mode_send == 0:
        mrd.flag_tarminal_mode_send = 2
        print("Set to send miniterminal data.")
    else:
        mrd.flag_tarminal_mode_send = 0
        #print("Stop to send miniterminal data.")
        for i in range(8):
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][0] = -1
            # 該当しないデータにはインデックスに-1を指定して送信データに反映されないようにしておく
            mrd.s_minitermnal_keep[i][1] = 0
            
# [Mini Terminal] ウィンドウのset&sendボタン処理
def set_and_send_miniterminal_data():  # ミニターミナルのセットボタンが押下されたら送信データをセットする
    set_miniterminal_data()
    set_tarminal_send_on()
    mrd.flag_send_miniterminal_data_once = 1    # ミニターミナルの値を1回送信する
    
    
            
# [Mini Terminal] ウィンドウのFlow, Step 切り替えラジオボタン処理
def set_transaction_mode(sender, app_data):
    if app_data == "Flow":  # ボタン押下でset_flowフラグをオン
        mrd.flag_set_flow_or_step = 2
        # mrd.flag_flow_switch = True
        mrd.flag_stop_flow = False
        print("Set flow to Meridian.")
    elif app_data == "Step":  # ボタン押下でset_stepフラグをオン
        mrd.flag_set_flow_or_step = -2
        # mrd.flag_flow_switch = True
        mrd.flag_stop_flow = True
        print("Set step to Meridian.")

# [Mini Terminal] ウィンドウの Next frame ボタン処理
def send_data_step_frame():  # チェックボックスに従いアクション送信フラグをオンオフ
    mrd.flag_stop_flow = False
    #mrd.flag_allow_flow = True
    print("Return: Send data and step to the next frame.")

#
def redis_sub(sender, app_data):
    print(f"[Debug] Redis checkbox clicked. Current flag_redis_sub: {mrd.flag_redis_sub}")

    if mrd.flag_redis_sub:
        print("[Debug] Stopping Redis subscription.")
        mrd.flag_redis_sub = False
    else:
        print("[Debug] Starting Redis subscription.")
        mrd.flag_redis_sub = True

# ================================================================================================================
# ---- dearpyguiによるコンソール画面描写 -----------------------------------------------------------------------------
# ================================================================================================================

def main():
    while (True):

# dpg描画処理1 ==========================================================
        dpg.create_context()
        dpg.create_viewport(title=TITLE_VERSION, width=853, height=540)

# ------------------------------------------------------------------------
# [ Axis Monitor ] : サーボ位置モニタリング用のウィンドウ（表示位置:上段/左側）
# ------------------------------------------------------------------------
        with dpg.window(label="Axis Monitor", width=250, height=370, pos=[5, 5]):
            with dpg.group(label='RightSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID R"+str(i), label="R"+str(i), max_value=100, min_value=-100, callback=set_servo_angle, pos=[10, 35+i*20], width=80)
            with dpg.group(label='LeftSide'):
                for i in range(0, 15, 1):
                    dpg.add_slider_float(default_value=0, tag="ID L"+str(i), label="L"+str(i), max_value=100, min_value=-100, callback=set_servo_angle, pos=[135, 35+i*20], width=80)
            dpg.add_button(label="Home", callback=set_servo_home, pos=[10, 340])  # Sendと書いてあるボタンをwindowの右下に設置
            dpg.add_radio_button(label="display_mode", items=["Target", "Actual"], callback=change_display_mode, default_value="Actual", pos=[90, 340], horizontal=True)

# ------------------------------------------------------------------------
# [ Message ] : メッセージ表示用ウィンドウ（表示位置:下段/左側）
# ------------------------------------------------------------------------
        with dpg.window(label="Messege", width=590, height=155, pos=[5, 380]):

            dpg.add_text("disp_send ", pos=[383, 53])
            dpg.add_checkbox(tag="disp_send", callback=set_disp_send, pos=[452, 53])
            dpg.add_text("disp_rcvd ", pos=[483, 53])
            dpg.add_checkbox(tag="disp_rcvd", callback=set_disp_rcvd, pos=[551, 53])
            dpg.add_button(label="ResetCycle",callback=reset_cycle, width=80, pos=[390, 28])
            dpg.add_button(label="ResetCounter",callback=reset_counter, width=90, pos=[480, 28])
            dpg.add_text(mrd.message0, tag="DispMessage0")
            dpg.add_text(mrd.message1, tag="DispMessage1")
            dpg.add_text(mrd.message2, tag="DispMessage2")
            dpg.add_text(mrd.message3, tag="DispMessage3")
            dpg.add_text(mrd.message4, tag="DispMessage4")

# ------------------------------------------------------------------------
# [ Sensor Monitor ] : センサー値モニタリング用ウィンドウ（表示位置:上段/中央）
# ------------------------------------------------------------------------
        with dpg.window(label="Sensor Monitor", width=335, height=175, pos=[260, 5]):
            with dpg.group(label='LeftSide'):
                dpg.add_slider_float(default_value=0, tag="mpu0", label="ac_x", max_value=327, min_value=-327, pos=[10, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu1", label="ac_y", max_value=327, min_value=-327, pos=[115, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu2", label="ac_z", max_value=327, min_value=-327, pos=[220, 35], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu3", label="gr_x", max_value=327, min_value=-327, pos=[10, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu4", label="gr_y", max_value=327, min_value=-327, pos=[115, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu5", label="gr_z", max_value=327, min_value=-327, pos=[220, 55], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu6", label="mg_x", max_value=327, min_value=-327, pos=[10, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu7", label="mg_y", max_value=327, min_value=-327, pos=[115, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu8", label="mg_z", max_value=327, min_value=-327, pos=[220, 75], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu9", label="_temp_int16", max_value=327, min_value=-327, pos=[10, 95], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu10", label="rol", max_value=327, min_value=-327, pos=[10, 120], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu11", label="pit", max_value=327, min_value=-327, pos=[115, 120], width=60)
                dpg.add_slider_float(default_value=0, tag="mpu12", label="yaw", max_value=327, min_value=-327, pos=[220, 120], width=60)
                dpg.add_button(label="SetYaw", callback=set_yaw_center, width=50, pos=[270, 148])

# ------------------------------------------------------------------------
# [ Command ] : コマンド送信/リモコン値表示用ウィンドウ（表示位置:中段/中央）
# ------------------------------------------------------------------------
        with dpg.window(label="Command", width=335, height=190, pos=[260, 185]):
            dpg.add_checkbox(label="Power", tag="Power", callback=set_servo_power, pos=[100, 27])
            dpg.add_checkbox(label="Demo", tag="Action", callback=set_demo_action, pos=[100, 53])
            dpg.add_checkbox(label="Python", tag="python", callback=set_python_action, pos=[100, 76])
            dpg.add_checkbox(label="Enable", tag="Enable", callback=set_enable, pos=[100, 99])

            dpg.add_text("ESP32 ->", pos=[20, 40])
            dpg.add_text("ESP32 <-", pos=[20, 83])

            dpg.add_checkbox(tag="ROS1pub", callback=ros1_pub, pos=[265, 40])
            dpg.add_text("-> ROS1", pos=[210, 40])
            dpg.add_checkbox(tag="ROS1sub", callback=ros1_sub, pos=[265, 83])
            dpg.add_text("<- ROS1", pos=[210, 83])
            dpg.add_checkbox(tag="Redis", callback=redis_sub, pos=[270, 104])
            dpg.add_text("<- Redis", pos=[210, 104])

            dpg.add_checkbox(tag="ros1_output_mode", callback=change_ros1_output_mode, user_data=1, pos=[305, 62])
            dpg.add_text("targ/rcvd", pos=[236, 62])

            dpg.draw_rectangle(pmin=[80, -4], pmax=[190, 95], color=(100, 100, 100, 255), thickness=1.0, fill=(0, 0, 0, 0))
            dpg.draw_line(p1=[84, 22], p2=[186, 22], color=(100, 100, 100, 255), thickness=1.0)

            dpg.add_text("Control Pad Monitor", pos=[10, 123])
            dpg.add_text("button", tag="pad_button", pos=[170, 123])
            dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx", max_value=127, min_value=-127, pos=[10, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly", max_value=127, min_value=-127, pos=[90, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx", max_value=127, min_value=-127, pos=[170, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry", max_value=127, min_value=-127, pos=[250, 143], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v", max_value=255, min_value=0, pos=[90, 163], width=40)
            dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v", max_value=255, min_value=0, pos=[170, 163], width=40)

# ------------------------------------------------------------------------
# [ Button Input ] : リモコン入力コンパネ用ウィンドウ（表示位置:上段/右側）
# ------------------------------------------------------------------------
        with dpg.window(label="Button Input", width=248, height=155, pos=[600, 5]):
            dpg.add_checkbox(tag="Btn_L2", callback=pad_btn_panel_on, user_data=256, pos=[15, 38])
            dpg.add_checkbox(tag="Btn_L1", callback=pad_btn_panel_on, user_data=1024, pos=[15, 60])
            dpg.add_checkbox(tag="Btn_L_UP", callback=pad_btn_panel_on, user_data=16, pos=[42, 80])
            dpg.add_checkbox(tag="Btn_L_DOWN", callback=pad_btn_panel_on, user_data=64, pos=[42, 124])
            dpg.add_checkbox(tag="Btn_L_LEFT", callback=pad_btn_panel_on, user_data=128, pos=[20, 102])
            dpg.add_checkbox(tag="Btn_L_RIGHT", callback=pad_btn_panel_on, user_data=32, pos=[64, 102])
            dpg.add_checkbox(tag="Btn_SELECT", callback=pad_btn_panel_on, user_data=1, pos=[100, 102])
            dpg.add_checkbox(tag="Btn_START", callback=pad_btn_panel_on, user_data=8, pos=[130, 102])
            dpg.add_checkbox(tag="Btn_R2", callback=pad_btn_panel_on, user_data=512, pos=[215, 38])
            dpg.add_checkbox(tag="Btn_R1", callback=pad_btn_panel_on, user_data=2048, pos=[215, 60])
            dpg.add_checkbox(tag="Btn_R_UP", callback=pad_btn_panel_on, user_data=4096, pos=[188, 80])
            dpg.add_checkbox(tag="Btn_R_DOWN", callback=pad_btn_panel_on, user_data=16384, pos=[188, 124])
            dpg.add_checkbox(tag="Btn_R_LEFT", callback=pad_btn_panel_on, user_data=32768, pos=[166, 102])
            dpg.add_checkbox(tag="Btn_R_RIGHT", callback=pad_btn_panel_on, user_data=8192, pos=[210, 102])

# ------------------------------------------------------------------------
# [ Mini Terminal ] : コマンド送信用ミニターミナル（表示位置:中段/右側）
# ------------------------------------------------------------------------
        with dpg.window(label="Mini Terminal", width=248, height=203, pos=[600, 165]):
            # with dpg.group(label='LeftSide'):
            dpg.add_text("Index", pos=[15, 25])
            dpg.add_text("Data", pos=[60, 25])
            dpg.add_input_text(tag="s_index0", decimal=True, default_value="0", width=40, pos=[15, 45])
            dpg.add_input_text(tag="s_data0", decimal=True, default_value=str(MSG_SIZE), width=60, pos=[60, 45])
            dpg.add_input_text(tag="s_index1", decimal=True, default_value="", width=40, pos=[15, 70])
            dpg.add_input_text(tag="s_data1", decimal=True, default_value="", width=60, pos=[60, 70])
            dpg.add_input_text(tag="s_index2", decimal=True, default_value="", width=40, pos=[15, 95])
            dpg.add_input_text(tag="s_data2", decimal=True, default_value="", width=60, pos=[60, 95])
            dpg.add_input_text(tag="s_index3", decimal=True, default_value="", width=40, pos=[15, 120])
            dpg.add_input_text(tag="s_data3", decimal=True, default_value="", width=60, pos=[60, 120])
            dpg.add_text("Index", pos=[130, 25])
            dpg.add_text("Data", pos=[175, 25])
            dpg.add_input_text(tag="s_index4", decimal=True, default_value="", width=40, pos=[130, 45])
            dpg.add_input_text(tag="s_data4", decimal=True, default_value="", width=60, pos=[175, 45])
            dpg.add_input_text(tag="s_index5", decimal=True, default_value="", width=40, pos=[130, 70])
            dpg.add_input_text(tag="s_data5", decimal=True, default_value="", width=60, pos=[175, 70])
            dpg.add_input_text(tag="s_index6", decimal=True, default_value="", width=40, pos=[130, 95])
            dpg.add_input_text(tag="s_data6", decimal=True, default_value="", width=60, pos=[175, 95])
            dpg.add_input_text(tag="s_index7", decimal=True, default_value="", width=40, pos=[130, 120])
            dpg.add_input_text(tag="s_data7", decimal=True, default_value="", width=60, pos=[175, 120])
            dpg.add_button(label="Set", callback=set_miniterminal_data, pos=[136, 148])
            dpg.add_button(label="Set&Send", callback=set_and_send_miniterminal_data, pos=[171, 148])
            dpg.add_text("Continuous ", pos=[140, 175])
            dpg.add_checkbox(tag="SendContinuously",callback=set_tarminal_continuous_on, pos=[215, 175])
            dpg.add_radio_button(["Flow", "Step"], tag="transaction_mode", pos=[10, 148], callback=set_transaction_mode, default_value="Flow", horizontal=True)
            dpg.add_button(label=" Next frame ", pos=[15, 175], callback=send_data_step_frame)  # 右下に設置

# dpg描画処理2 =========================================================
        with dpg.value_registry(): # dpg変数値の登録
            dpg.add_int_value(tag="button_data")

        dpg.setup_dearpygui()
        dpg.show_viewport()

# dpg描画内容のデータ更新 ================================================
        while dpg.is_dearpygui_running():
            signal.signal(signal.SIGINT, signal.SIG_DFL)

            # メッセージ欄の表示更新
            dpg.set_value("DispMessage0", mrd.message0)
            dpg.set_value("DispMessage1", mrd.message1)
            dpg.set_value("DispMessage2", mrd.message2)
            dpg.set_value("DispMessage3", mrd.message3)
            dpg.set_value("DispMessage4", mrd.message4)

            # サーボデータとIMUデータの表示更新
            for i in range(0, 15, 1):
                _idld = mrd.d_meridim[21+i*2]
                _idrd = mrd.d_meridim[51+i*2]
                _idsensor = mrd.r_meridim[i+2]/10000
                dpg.set_value("ID L"+str(i), _idld/100)  # サーボIDと数値の表示
                dpg.set_value("ID R"+str(i), _idrd/100)

                if i < 13:  # IMUデータの更新
                    if i < 11:
                        dpg.set_value("mpu"+str(i), _idsensor)
                    else:
                        dpg.set_value("mpu"+str(i), _idsensor*100)

            # リモコンデータの表示更新
            pad_button_short = np.array([0], dtype=np.uint16)
            # 受信値とコンソール入力値を合成
            pad_button_short[0] = mrd.r_meridim[15] | mrd.pad_button_panel_short[0]
            dpg.set_value("pad_button", str(pad_button_short[0]))
            dpg.set_value("pad_Lx", int(mrd.r_meridim_char[33]))
            dpg.set_value("pad_Ly", int(mrd.r_meridim_char[32]))
            dpg.set_value("pad_Rx", int(mrd.r_meridim_char[35]))
            dpg.set_value("pad_Ry", int(mrd.r_meridim_char[34]))
            
            _padL2val_bin = struct.pack('b', mrd.r_meridim_char[36])
            _padL2val = struct.unpack('B', _padL2val_bin)[0]
            _padR2val_bin = struct.pack('b', mrd.r_meridim_char[37])
            _padR2val = struct.unpack('B', _padR2val_bin)[0]
            
            dpg.set_value("pad_L2v", int(_padL2val))
            dpg.set_value("pad_R2v", int(_padR2val))
            dpg.set_value("button_data", int(mrd.r_meridim[15]))

# ROS1 joint_statesのパブリッシュ =======================================
            global rospy_imported
            if mrd.flag_ros1_pub:  # ROS送信:joint_statesのpublishを実施
                if rospy_imported:
                    if mrd.flag_ros1 == 0:
                        rospy.init_node('joint_state_meridim', anonymous=True)
                        flag_ros1 = 1
                    joint_pub = rospy.Publisher(
                        'joint_states', JointState, queue_size=10)
                    rate = rospy.Rate(100)  # 100hz
                    js_meridim = JointState()
                    js_meridim.header.stamp = rospy.Time.now()
                    js_meridim.name =\
                        ['c_head_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',
                         'l_elbow_pitch',                 'l_hipjoint_yaw',                  'l_hipjoint_roll',                  'l_hipjoint_pitch',
                         'l_knee_pitch',              'l_ankle_pitch',                   'l_ankle_roll',
                         'c_chest_yaw',            'r_shoulder_pitch',                'r_shoulder_roll',                  'r_elbow_yaw',
                         'r_elbow_pitch',     'r_hipjoint_yaw',                  'r_hipjoint_roll',                  'r_hipjoint_pitch',
                         'r_knee_pitch',  'r_ankle_pitch',                   'r_ankle_roll']
                    js_meridim.position = \
                        [math.radians(mrd.s_meridim_motion_f[21]/100*mrd.jspn[0]), math.radians(mrd.s_meridim_motion_f[23]/100*mrd.jspn[1]), 
                         math.radians(mrd.s_meridim_motion_f[25]/100)*mrd.jspn[2], math.radians(mrd.s_meridim_motion_f[27]/100*mrd.jspn[3]),
                         math.radians(mrd.s_meridim_motion_f[29]/100*mrd.jspn[4]), math.radians(mrd.s_meridim_motion_f[31]/100*mrd.jspn[5]), 
                         math.radians(mrd.s_meridim_motion_f[33]/100*mrd.jspn[6]), math.radians(mrd.s_meridim_motion_f[35]/100*mrd.jspn[7]),
                         math.radians(mrd.s_meridim_motion_f[37]/100*mrd.jspn[8]), math.radians(mrd.s_meridim_motion_f[39]/100*mrd.jspn[9]), 
                         math.radians(mrd.s_meridim_motion_f[41]/100*mrd.jspn[10]), math.radians(mrd.s_meridim_motion_f[51]/100*mrd.jspn[15]), 
                         math.radians(mrd.s_meridim_motion_f[53]/100*mrd.jspn[16]), math.radians(mrd.s_meridim_motion_f[55]/100*mrd.jspn[17]), 
                         math.radians(mrd.s_meridim_motion_f[57]/100*mrd.jspn[18]), math.radians(mrd.s_meridim_motion_f[59]/100*mrd.jspn[19]), 
                         math.radians(mrd.s_meridim_motion_f[61]/100*mrd.jspn[20]), math.radians(mrd.s_meridim_motion_f[63]/100*mrd.jspn[21]), 
                         math.radians(mrd.s_meridim_motion_f[65]/100*mrd.jspn[22]), math.radians(mrd.s_meridim_motion_f[67]/100*mrd.jspn[23]),
                         math.radians(mrd.s_meridim_motion_f[69]/100*mrd.jspn[24]), math.radians(mrd.s_meridim_motion_f[71]/100*mrd.jspn[25])]

                    js_meridim.velocity = []
                    # js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    js_meridim.effort = []
                    joint_pub.publish(js_meridim)
                    rate.sleep()
                else:
                    print("ROS is not avaliable.")

# ROS1 joint_statesのサブスクライブ =====================================
            if mrd.flag_ros1_sub:
                if rospy_imported:
                    # joint_sub = rospy.Subscriber('joint_states', JointState, queue_size=10)
                    if flag_ros1 == 0:
                        rospy.init_node('joint_state_meridim', anonymous=True)
                        flag_ros1 = 1
                    rospy.Subscriber('joint_states', JointState,
                                     joinstate_to_meridim)
                    # rospy.spin()
                else:
                    print("ROS is not avaliable.")
                    
# =====================================================================
            # dpg表示更新処理
            dpg.render_dearpygui_frame()

            time.sleep(0.003)  # CPUの負荷を下げる
        dpg.destroy_context()

# ================================================================================================================
# ---- スレッド処理 ------------------------------------------------------------------------------------------------
# ================================================================================================================
if __name__ == '__main__': # スレッド2つで送受信と画面描写を並列処理
    thread1 = threading.Thread(target=meridian_loop) # サブスレッドでフラグ監視・通信処理・計算処理
    thread1.start()
    main()  # メインスレッドでdearpygui描写
