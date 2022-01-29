// カリカリチューン動く！ ESP32DevKitC - (Wifi/UDP) - PC/python  通信
// ESP32用

//[E-2] ライブラリ導入 -----------------------------------
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
int checksum; //チェックサム計算用
long frame_count = 0;
long error_count_udp = 0;

//wifi設定
WiFiUDP udp;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_spi_meridim; //SPI受信用共用体のインスタンスを宣言
UnionData r_spi_meridim; //SPI受信用共用体のインスタンスを宣言
UnionData s_udp_meridim; //UDP送信用共用体のインスタンスを宣言
UnionData r_udp_meridim; //UDP受信用共用体のインスタンスを宣言

void setup()
{
  Serial.begin(2000000);
  delay(120);//シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println("Serial Start...");

  //WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  Serial.println("Connecting to WiFi to : " + String(AP_SSID));//接続先を表示
  delay(100);
  WiFi.begin(AP_SSID, AP_PASS);//Wifiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50);//接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected.");//WiFi接続完了通知
  Serial.print("ESP32's IP address is  => ");
  Serial.println(WiFi.localIP());//ESP32自身のIPアドレスの表示
  //ESP32自身のBluetoothMacアドレスを表示
  uint8_t bt_mac[6];
  String self_mac_address = "";

  esp_read_mac(bt_mac, ESP_MAC_BT);
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
  Serial.println();

  //UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

}


//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// ■ 受信用の関数 パケットが来ていれば、RECVDATANUM 分だけr_ufdata配列に保存 -----
void receiveUDP() {
  int packetSize = udp.parsePacket();
  byte tmpbuf[MSG_BUFF];

  //データの受信
  if (packetSize == MSG_BUFF) {
    udp.read(tmpbuf, MSG_BUFF);
    for (int i = 0; i < MSG_BUFF; i++)
    {
      r_udp_meridim.bval[i] = tmpbuf[i];
    }
  }
}

// ■ 送信用の関数 ----------------------------------------------------------
void sendUDP() {
  udp.beginPacket(SEND_IP, SEND_PORT);//UDPパケットの開始
  for (int i = 0; i < MSG_BUFF; i++) {
    udp.write(s_udp_meridim.bval[i]);//１バイトずつ送信
  }
  udp.endPacket();//UDPパケットの終了
}


//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //---- < 1 > U D P 受 信 --------------------------------------------------
  //[1-1] UDP受信の実行 もしデータパケットが来ていれば受信する
  receiveUDP();

  //[1-2] UDP受信配列からSPI送信配列にデータを転写
  for (int i = 0; i < MSG_BUFF; i++) {
    s_spi_meridim.bval[i] = r_udp_meridim.bval[i];
  }

  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += int(s_spi_meridim.sval[i]);
  }
  checksum = short(~checksum);//チェックサムを計算

  if (checksum == s_spi_meridim.sval[MSG_SIZE - 1])
  {
  } else {
    error_count_udp ++;
  }
  frame_count = frame_count + 1;
  Serial.print(error_count_udp);    Serial.print("/");    Serial.println(frame_count);

  //---- < 2 > S P I 送 信 信 号 作 成  -----------------------------------------------

  //---- < 3 > S P I 送 受 信 -----------------------------------------------

  //---- < 4 > U D P 送 信 信 号 作 成 --------------------------------------------------

  for (int i = 0; i < MSG_SIZE - 2; i++) //配列の末尾以外をデータを入れる
  {
    short rnd = random(-30000, 30000);
    s_udp_meridim.sval[i] = rnd;
  }

  // チェックサムを計算して格納
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += s_udp_meridim.sval[i];
  }
  checksum = short(~checksum);//チェックサムを計算
  s_udp_meridim.sval[MSG_SIZE - 1] = short(checksum);

  //---- < 5 > U D P 送 信 ----------------------------------------------
  //[4-1] UDP送信を実行
  sendUDP();
  delayMicroseconds(1);
  //delay(1);
}
