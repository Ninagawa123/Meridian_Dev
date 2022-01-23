# #!/usr/bin/python3
# coding: UTF-8
# もしくは　#!/usr/bin/env python など環境に合わせて

import numpy as np
import socket
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal

UDP_RESV_IP="192.168.1.xx" #このPCのIPアドレス
UDP_RESV_PORT=22222 #受信ポート

UDP_SEND_IP="192.168.1.xx" #送信先のESP32のIPアドレス
UDP_SEND_PORT=22224 #送信ポート

MSG_SIZE = 90
MSG_BUFF = MSG_SIZE * 2

UPDATE_YAW_CENTER_FLAG = 0
UPDATE_YAW_CENTER_NUM = 1002

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)#UDP用のsocket設定
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

data2 = []
loop_count = 0
error_count = 0
flag_stop = 0

r_short_data_disp=list(range(MSG_SIZE))
r_short_data_disp_char=list(range(MSG_SIZE*2))
message0 = "This PC's IP adress is "+UDP_RESV_IP
message1 = ""
button = "button"

#データの送受信
def main():
    global message0
    global message1

    while (True):
        message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:

                global loop_count
                global r_short_data_disp
                global r_short_data_disp_char
                loop_count += 1
                r_bin_data,addr = sock.recvfrom(1472)
                r_short_data=struct.unpack('90h',r_bin_data)
                r_short_data_disp_char=struct.unpack('180b',r_bin_data)
                message1 = "UDP data receiving from "+UDP_SEND_IP+"."

                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-2):
                    checksum[0] += r_short_data[i]
                    r_short_data_disp[i]=r_short_data[i]
                checksum[0] = ~checksum[0] & 0xffff


                if checksum[0] == r_short_data[MSG_SIZE-1]:
                    pass
                else:
                    global error_count        
                    error_count += 1
                signal.signal(signal.SIGINT, signal.SIG_DFL)

                #ここで送信データを作成する
                s_short_data=[]
                s_short_data=list(r_short_data)

                #ヨー軸センターリセットコマンド
                global UPDATE_YAW_CENTER_FLAG
                if (UPDATE_YAW_CENTER_FLAG > 0):
                    UPDATE_YAW_CENTER_FLAG -= 1 
                    s_short_data[0] = UPDATE_YAW_CENTER_NUM
                    if (UPDATE_YAW_CENTER_FLAG==0):
                        print("Set Yaw Center. COMMAND: "+str(UPDATE_YAW_CENTER_NUM))
                    
                #格納した送信データについてチェックサムを追加
                checksum[0] = 0
                for i in  range(MSG_SIZE-2):
                    checksum[0] += s_short_data[i]
                checksum[0] = ~checksum[0] & 0xffff
                s_short_data[MSG_SIZE-1]=checksum[0]

                #データをパックしてUDP送信
                s_bin_data=struct.pack('90h',*s_short_data)
                sock.sendto(s_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))


#dearpyguiによるコンソール画面描写
def dpgrun():

    def set_yaw_center():#ターミナルにClicked!と表示する
        global UPDATE_YAW_CENTER_FLAG
        UPDATE_YAW_CENTER_FLAG = 10

    dpg.create_context()
    dpg.create_viewport(title='Meridian Console with ROS', width=600, height=480)
    with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
        with dpg.group(label='LeftSide'): 
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=100,min_value=-100,pos=[10,35+i*20], width=80)
        with dpg.group(label='RightSide'):
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=100,min_value=-100,pos=[135,35+i*20], width=80)

    with dpg.window(label="Messege", width=590, height=115,pos=[5,360]):
        dpg.add_text(message0,tag="DispMessage0")
        dpg.add_text(message1,tag="DispMessage1")

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

    with dpg.value_registry():
        dpg.add_int_value(tag="button_data")

    with dpg.window(label="Command", width=335, height=170,pos=[260,185]):
        dpg.add_text("Control Pad Monitor", pos=[10,100])
        dpg.add_text("button",tag="pad_button", pos=[170,100])
        dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx",max_value=127,min_value=-127, pos=[10,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly",max_value=127,min_value=-127, pos=[90,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx",max_value=127,min_value=-127, pos=[170,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry",max_value=127,min_value=-127, pos=[250,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v",max_value=255,min_value=0, pos=[90,140], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v",max_value=255,min_value=0, pos=[170,140], width=40)

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        for i in range(0, 15, 1):
            global button
            idld = r_short_data_disp[21+i*2]
            idrd = r_short_data_disp[51+i*2]
            idsensor = r_short_data_disp[i+2]/10000
            dpg.set_value("ID L"+str(i), idld/100)
            dpg.set_value("ID R"+str(i), idrd/100)
            dpg.set_value("DispMessage0", message0)
            dpg.set_value("DispMessage1", message1)
            if i < 13:
                if i < 11:
                    dpg.set_value("mpu"+str(i),idsensor)
                else:
                    dpg.set_value("mpu"+str(i),idsensor*100)


        dpg.set_value("pad_button", str(r_short_data_disp[80]))
        dpg.set_value("pad_Lx", r_short_data_disp_char[163])
        dpg.set_value("pad_Ly", r_short_data_disp_char[162])
        dpg.set_value("pad_Rx", r_short_data_disp_char[165])
        dpg.set_value("pad_Ry", r_short_data_disp_char[164])
        padL2val = (r_short_data_disp_char[167])
        if (padL2val<0):
            padL2val = 256+padL2val
        if (r_short_data_disp[80]&256==0):
            padL2val = 0
        padR2val = (r_short_data_disp_char[166])
        if (padR2val<0):
            padR2val = 256+padR2val
        if (r_short_data_disp[80]&512==0):
            padR2val = 0
        dpg.set_value("pad_L2v", padL2val)
        dpg.set_value("pad_R2v", padR2val)
        dpg.set_value("button_data", r_short_data_disp[80])

        dpg.render_dearpygui_frame()

    dpg.destroy_context()

#スレッド2つで送受信と画面描写を並列処理
if __name__ == '__main__':
    thread1 = threading.Thread(target=dpgrun)
    thread1.start()
    main()
