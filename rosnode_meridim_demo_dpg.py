#!/usr/bin/python3
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

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)#UDP用のsocket設定
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

data2 = []
loop_count = 0
error_count = 0

flag_stop = 0

r_short_data_disp=list(range(MSG_SIZE))
message = ""

def main():
    global message
    while not rospy.is_shutdown():
        print("Waiting for UDP signal from", UDP_SEND_IP, "...")    
        message = "Waiting for UDP signal from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:

                global loop_count
                global r_short_data_disp
                loop_count += 1
                r_bin_data,addr = sock.recvfrom(1472)
                sock.sendto(r_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))
                r_short_data=struct.unpack('90h',r_bin_data)
                #print(r_short_data)
                message = "UDP signal receiving."

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
                    global error_count        
                    error_count += 1
                signal.signal(signal.SIGINT, signal.SIG_DFL)
                #print("COUNT:",loop_count," ERROR:",error_count," ErrorRate:",'{:.02f}'.format(error_count/loop_count*100),"%")

def dpgrun():
    dpg.create_context()

    dpg.create_viewport(title='Meridian Control Panel DPG', width=600, height=480)
    with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
        with dpg.group(label='LeftSide'): 
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=120,min_value=-120,pos=[10,35+i*20], width=80)
        with dpg.group(label='RightSide'):
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=120,min_value=-120,pos=[135,35+i*20], width=80)

    with dpg.window(label="Messege", width=590, height=115,pos=[5,360]):
        dpg.add_text(message,tag="DispMessage")

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        for i in range(0, 15, 1):
            idld = r_short_data_disp[21+i*2]
            idrd = r_short_data_disp[51+i*2]
            dpg.set_value("ID L"+str(i), idld/100)
            dpg.set_value("ID R"+str(i), idrd/100)
            dpg.set_value("DispMessage", message)
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
