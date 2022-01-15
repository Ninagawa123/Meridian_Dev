#!/usr/bin/python3
#もしくは　#!/usr/bin/env python など環境に合わせて
# カリカリチューン動く！ ESP32DevKitC - (Wifi/UDP) - PC/python
# PC/python用

from random import random
import numpy as np
import socket
from contextlib import closing
import struct
import signal
import time
import random

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
start = time.time()
r_upd_meridim_short=list(range(MSG_SIZE))
r_upd_meridim_char=list(range(MSG_SIZE*2))
s_upd_meridim_short=list(range(MSG_SIZE))
s_upd_meridim_char=list(r_upd_meridim_char)

while True:
    with closing(sock):
        print("Waitind for UDP signal from", UDP_SEND_IP, "...") #起動メッセージ
        while True:
            loop_count += 1

            #UDP受信
            r_bin_data,addr = sock.recvfrom(1472)
            r_upd_meridim_tuple=struct.unpack('90h',r_bin_data)
            print(r_upd_meridim_tuple)

            #UDPチェックサム
            checksum = np.array([0], dtype=np.int16)
            for i in  range(MSG_SIZE-2):
                checksum[0] += r_upd_meridim_tuple[i]
            checksum[0] = ~checksum[0] & 0xffff
            print("[Calc] ",checksum[0])
            print("[Ans ] ",r_upd_meridim_tuple[MSG_SIZE-1])
            
            if checksum[0] == r_upd_meridim_tuple[MSG_SIZE-1]:
                pass
            else:
                #print("DATA ERROR")
                error_count += 1
            
            #UDP送信データ作成(一旦受信データを転記)
            for i in  range(MSG_SIZE-1):
                s_upd_meridim_short[i] = r_upd_meridim_tuple[i]

            #UDP送信データ作成(ランダムなデータを記入)
            for i in  range(MSG_SIZE-2):
                s_upd_meridim_short[i] = random.randint(-30000,30000)

            #格納した送信データについてチェックサムを追加
            checksum[0] = 0
            for i in  range(MSG_SIZE-2):
                checksum[0] += s_upd_meridim_short[i]
            checksum[0] = ~checksum[0] & 0xffff
            s_upd_meridim_short[MSG_SIZE-1]=checksum[0]

            #データをパックしてUDP送信
            s_bin_data=struct.pack('90h',*s_upd_meridim_short)
            sock.sendto(s_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))

            signal.signal(signal.SIGINT, signal.SIG_DFL)
            
            now = time.time()-start
            print(error_count,"/",loop_count,"  error",error_count/loop_count*100,"% ", int(loop_count/now),"Hz")

if __name__ == '__main__':
    main()
