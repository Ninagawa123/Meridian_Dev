# #!/usr/bin/python3
# coding: UTF-8
#もしくは　#!/usr/bin/env python など環境に合わせて
#from _typeshed import IdentityFunction

from re import I

from yaml.tokens import TagToken
import rospy
from sensor_msgs.msg import JointState
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
UPDATE_YAW_CENTER_NUM = 102

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)#UDP用のsocket設定
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

data2 = []
loop_count = 0
error_count = 0

flag_stop = 0

r_short_data_disp=list(range(MSG_SIZE))
message0 = "This PC's IP adress is "+UDP_RESV_IP
message1 = ""
button = "button"

def main():
    global message
    global message0
    global message1

    while not rospy.is_shutdown():
        message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:

                global loop_count
                global r_short_data_disp
                loop_count += 1
                r_bin_data,addr = sock.recvfrom(1472)
                #sock.sendto(r_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))
                r_short_data=struct.unpack('90h',r_bin_data)
                #print(r_short_data)
                message1 = "UDP data receiving from "+UDP_SEND_IP+"."

                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-2):
                    checksum[0] += r_short_data[i]
                    r_short_data_disp[i]=r_short_data[i]
                checksum[0] = ~checksum[0] & 0xffff
                #print("[Calc] ",checksum[0])
                #print("[Ans ] ",r_short_data[MSG_SIZE-1])
                #r_short_data[MSG_SIZE-1]=checksum[0]

                if checksum[0] == r_short_data[MSG_SIZE-1]:
                    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
                    rospy.init_node('joint_state_publisher_meridim')
                    rate = rospy.Rate(500) # 500hz
                    js_meridim = JointState()
                    js_meridim.header.stamp = rospy.Time.now()
                    js_meridim.name =     ['c_chest_yaw',                      'l_shoulder_pitch',                'l_shoulder_roll',                  'l_elbow_yaw',                      'l_elbow_pitch',                    'l_hipjoint_yaw',                   'l_hipjoint_roll',                  'l_hipjoint_pitch',                 'l_knee_pitch',                     'l_ankle_pitch',                    'l_ankle_roll',                     'c_head_yaw',                       'r_shoulder_pitch',                  'r_shoulder_roll',                   'r_elbow_yaw',                      'r_elbow_pitch',                     'r_hipjoint_yaw',                    'r_hipjoint_roll',                  'r_hipjoint_pitch',                 'r_knee_pitch',                      'r_ankle_pitch',                     'r_ankle_roll']
                    js_meridim.position = [math.radians(r_short_data[21]/100), math.radians(r_short_data[23]/100), math.radians(r_short_data[25]/100), math.radians(r_short_data[27]/100), math.radians(r_short_data[29]/100), math.radians(r_short_data[31]/100), math.radians(r_short_data[33]/100), math.radians(r_short_data[35]/100), math.radians(r_short_data[37]/100), math.radians(r_short_data[39]/100), math.radians(r_short_data[41]/100), math.radians(r_short_data[51]/100), math.radians(r_short_data[53]/100), -math.radians(r_short_data[55]/100), -math.radians(r_short_data[57]/100), math.radians(r_short_data[59]/100), -math.radians(r_short_data[61]/100), -math.radians(r_short_data[63]/100), math.radians(r_short_data[65]/100), math.radians(r_short_data[67]/100),  math.radians(r_short_data[69]/100), -math.radians(r_short_data[71]/100)]
                    js_meridim.velocity = []
                    #js_meridim.velocity = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                    js_meridim.effort = []        
                    joint_pub.publish(js_meridim)
                    rate.sleep() 
                else:
                    global error_count        
                    error_count += 1
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                #print("COUNT:",loop_count," ERROR:",error_count," ErrorRate:",'{:.02f}'.format(error_count/loop_count*100),"%")
                #print("PAD:",r_short_data_disp[80])

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
        print("Clicked!")#ターミナルにClicked!と表示する

    dpg.create_context()
    dpg.create_viewport(title='Meridian Control Panel DPG', width=600, height=480)
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
            idsensor = r_short_data_disp[i+2]/100
            dpg.set_value("ID L"+str(i), idld/100)
            dpg.set_value("ID R"+str(i), idrd/100)
            dpg.set_value("DispMessage0", message0)
            dpg.set_value("DispMessage", message1)
            if i < 13:
                dpg.set_value("mpu"+str(i),idsensor)
        dpg.set_value("pad_button", str(r_short_data_disp[80]))
        dpg.set_value("pad_Lx",-(r_short_data_disp[81]>>8&0x00ff & 0b10000000) | (r_short_data_disp[81]>>8&0x00ff & 0b01111111))
        dpg.set_value("pad_Ly", -(r_short_data_disp[81]&0x00ff & 0b10000000) | (r_short_data_disp[81]&0x00ff & 0b01111111))
        dpg.set_value("pad_Rx", -(r_short_data_disp[82]>>8&0x00ff & 0b10000000) | (r_short_data_disp[82]>>8&0x00ff & 0b01111111))
        dpg.set_value("pad_Ry", -(r_short_data_disp[82]&0x00ff & 0b10000000) | (r_short_data_disp[82]&0x00ff & 0b01111111))
        padl2val = -(r_short_data_disp[83]>>8&0x00ff & 0b10000000) | (r_short_data_disp[83]>>8&0x00ff & 0b01111111)
        if (padl2val<0):
            padl2val = 256+padl2val
        if (r_short_data_disp[80]&256==0):
            padl2val = 0
        dpg.set_value("pad_L2v", padl2val)

        padR2val = -(r_short_data_disp[83]&0x00ff & 0b10000000) | (r_short_data_disp[83]&0x00ff & 0b01111111)
        if (padR2val<0):
            padR2val = 256+padR2val
        if (r_short_data_disp[80]&512==0):
            padR2val = 0
        dpg.set_value("pad_R2v", padR2val)
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
