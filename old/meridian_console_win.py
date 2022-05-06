#!/usr/bin/python3
#もしくは　#!/usr/bin/env python など環境に合わせて
#
#ライブラリのインストールは
#pip install dearpygui
#or
#pip3 install dearpygui


import numpy as np
import socket
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal

ip = socket.gethostbyname(socket.gethostname())#このPCのIPアドレスを自動取得
UDP_RESV_IP=ip #このPCのIPアドレスを設定
#UDP_RESV_IP="192.168.1.7" #このPCのIPアドレスを値で指定したい場合は上の２行をコメントアウトしこちらを使う
UDP_RESV_PORT=22222 #受信ポート

UDP_SEND_IP="192.168.1.24" #送信先のESP32のIPアドレス
UDP_SEND_PORT=22224 #送信ポート

MSG_SIZE = 90
MSG_BUFF = MSG_SIZE * 2

UPDATE_YAW_CENTER_FLAG = 0
UPDATE_YAW_CENTER_NUM = 102

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)#UDP用のsocket設定
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

data2 = []
loop_count = 0
error_count = 0

flag_stop = 0

r_short_data_disp=list(range(MSG_SIZE))
message0 = "This PC's IP adress is "+ip
message1 = ""

def main():
    global message0
    global message1
    while True:
        #print("Waiting for UDP signal from", UDP_SEND_IP, "...")    
        message1 = "Waiting for UDP signal from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:

                global loop_count
                global r_short_data_disp
                loop_count += 1
                r_bin_data,addr = sock.recvfrom(1472)
                #sock.sendto(r_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))
                r_short_data=struct.unpack('90h',r_bin_data)
                #print(r_short_data)
                message1 = "UDP data receiving."

                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-2):
                    checksum[0] += r_short_data[i]
                    r_short_data_disp[i]=r_short_data[i]
                checksum[0] = ~checksum[0] & 0xffff
                #print("[Calc] ",checksum[0])
                #print("[Ans ] ",r_short_data[MSG_SIZE-1])
                #r_short_data[MSG_SIZE-1]=checksum[0]

                if checksum[0] == r_short_data[MSG_SIZE-1]:
                    pass
                else:
                    global error_count        
                    error_count += 1
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                #print("COUNT:",loop_count," ERROR:",error_count," ErrorRate:",'{:.02f}'.format(error_count/loop_count*100),"%")

                #ここでデータを作成する


                s_short_data=[]
                s_short_data=list(r_short_data)

                global UPDATE_YAW_CENTER_FLAG
                #global UPDATE_YAW_CENTER_NUM

                if (UPDATE_YAW_CENTER_FLAG ==1):
                    UPDATE_YAW_CENTER_FLAG = 0
                    s_short_data[0] = UPDATE_YAW_CENTER_NUM


                #s_short_data[0]=102#テスト用ダミーデータ

                checksum[0] = 0
                for i in  range(MSG_SIZE-2):
                    checksum[0] += s_short_data[i]
                checksum[0] = ~checksum[0] & 0xffff
                s_short_data[MSG_SIZE-1]=checksum[0]

                s_bin_data=struct.pack('90h',*s_short_data)


                #r_bin_data,addr = sock.recvfrom(1472)
                #sock.sendto(r_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))
                #r_short_data=struct.unpack('90h',s_bin_data)


                sock.sendto(s_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))
                #sock.sendto(r_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))






def dpgrun():


    def set_yaw_center():
        global UPDATE_YAW_CENTER_FLAG
        UPDATE_YAW_CENTER_FLAG = 1
        #print("Clicked!")#ターミナルにClicked!と表示する

    dpg.create_context()
    dpg.create_viewport(title='Meridian Windows connection test', width=616, height=520)
    with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
        with dpg.group(label='LeftSide'): 
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=100,min_value=-100,pos=[10,35+i*20], width=80)
        with dpg.group(label='RightSide'):
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=100,min_value=-100,pos=[135,35+i*20], width=80)

    with dpg.window(label="Messege", width=590, height=115,pos=[5,360]):
        dpg.add_text(message0,tag="DispMessage0")
        dpg.add_text(message1,tag="DispMessage")

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

    with dpg.window(label="Command", width=335, height=170,pos=[260,185]):
        pass

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        for i in range(0, 15, 1):
            idld = r_short_data_disp[21+i*2]
            idrd = r_short_data_disp[51+i*2]
            idsensor = r_short_data_disp[i+2]/100
            dpg.set_value("ID L"+str(i), idld/100)
            dpg.set_value("ID R"+str(i), idrd/100)
            dpg.set_value("DispMessage0", message0)
            dpg.set_value("DispMessage", message1)
            if i < 13:
                dpg.set_value("mpu"+str(i),idsensor)
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

if __name__ == '__main__':
    thread1 = threading.Thread(target=dpgrun)
    thread1.start()
    main()
    
    #try:
    #    joint_publisher_func()
    #except rospy.ROSInterruptException:
    #    pass
