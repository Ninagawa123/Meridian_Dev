//Meridian_211122_for_ESP32_PathThrough
//ESP32_DevKitC

/*
  -------------------------------------------------------------------------
  ---- ESP32 ピンアサイン [E-1-0] --------------------------------------
  -------------------------------------------------------------------------
  [3V3]               -> 
  [EN]                -> 
  [VP,36]             -> 
  [VN,39]             -> 
  [34]                -> 
  [35]                -> 
  [32]                ->
  [33]                ->
  [25]                ->
  [26]                ->
  [27]                ->
  [14] SPI_CLK        -> SPI_SCK (Teensy[13])
  [12] SPI_MISO       -> SPI_MISO (Teensy[12])
  [GND]               ->
  [13] SPI_MOSI       -> SPI_MOSI (Teensy[11])
  [D2,9] RX1          ->
  [D3,10] TX1         ->
  [CMD,11]            ->
  [5V]                -> 5V
  [GND]               -> GND
  [23]                ->
  [22] I2C_SCL        ->
  [TX] TX0            ->
  [RX] RX0            ->
  [21] I2C_SDA        ->
  [GND]               ->
  [19]                ->
  [18]                ->
  [05]                ->
  [17] TX2            ->
  [16] RX2            ->
  [04]                ->
  [00]                ->
  [02]                ->
  [15] SPI_SS         -> SPI_CS (Teensy[10])
  [D1,8]              ->
  [D0,7] I2C_SCL      ->
  [CLK,6] I2C_SDA     ->
  -------------------------------------------------------------------------
  ---- Meridim配列 一覧表 [S-1-2] ------------------------------------------
  -------------------------------------------------------------------------
  [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
  [01]      移動時間
  [02]-[04] IMU:acc＿x,acc＿y,acc＿z    加速度x,y,z
  [05]-[07] IMU:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
  [08]-[10] IMU:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
  [11]      IMU:temp                   温度
  [12]-[14] IMU:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
  [15]      free
  [16]      free
  [17]      free
  [18]      free
  [19]      free
  [20]      サーボID LO  コマンド
  [21]      サーボID LO  データ値
  ...
  [48]      サーボID L14 コマンド
  [49]      サーボID L14 データ値
  [50]      サーボID RO  コマンド
  [51]      サーボID RO  データ値
  ...
  [78]      サーボID R14 コマンド
  [79]      サーボID R14 データ値
  [80]      free ボタンデータ1
  [81]      free ボタンデータ2
  [82]      free ボタンアナログ1
  [83]      free ボタンアナログ2
  [84]      free (ボタンアナログ3)
  [85]      free (ボタンアナログ4)
  [86]      free (Teensy SPI receive Error Rate)
  [87]      free (ESP32 SPI receive Error Rate)
  [88]      free (PC UDP receive Error Rate)
  [89]      チェックサム
*/

//[E-2] ライブラリ導入 -----------------------------------
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
#include <ESP32DMASPISlave.h>
ESP32DMASPI::Slave slave;

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.x.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
uint8_t* s_message_buf;
uint8_t* r_message_buf;

//wifi設定
WiFiUDP udp;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_upd_message_buf; //送信用共用体のインスタンスを宣言
UnionData r_upd_message_buf; //受信用共用体のインスタンスを宣言
UnionData r_spi_message_buf;

void setup()
{
  Serial.begin(2000000);
  delay(500);//シリアル準備待ち用ディレイ
  Serial.println("Serial Start...");

  //WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  Serial.println("Connecting to WiFi to : " + String(AP_SSID));//接続先を表示
  delay(100);
  WiFi.begin(AP_SSID, AP_PASS);//Wifiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(100);//接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected.");//WiFi接続完了通知
  Serial.print("WiFi connected. ESP32's IP address is : ");
  Serial.println(WiFi.localIP());//デバイスのIPアドレスの表示

  //UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

  //SPI通信用のDMA（大容量通信用メモリバッファ）の設定
  s_message_buf = slave.allocDMABuffer(MSG_BUFF);
  r_message_buf = slave.allocDMABuffer(MSG_BUFF);

  // バッファのリセット
  memset(s_message_buf, 0, MSG_BUFF);
  memset(r_message_buf, 0, MSG_BUFF);

  //SPI通信の初期設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF);
  slave.setDMAChannel(2);
  slave.setQueueSize(1);
  slave.begin();
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
      r_upd_message_buf.bval[i] = tmpbuf[i];
    }
  }
}

// ■ 送信用の関数 ----------------------------------------------------------
void sendUDP() {
  String test = "";//表示用の変数

  udp.beginPacket(SEND_IP, SEND_PORT);//UDPパケットの開始

  for (int i = 0; i < MSG_BUFF; i++) {
    udp.write(s_upd_message_buf.bval[i]);//１バイトずつ送信
    if (i % 2 == 0) {//表示用に送信データを共用体から2バイトずつ取得
      test += String(s_upd_message_buf.sval[i / 2]) + ", ";
    }
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
    s_message_buf[i] = r_upd_message_buf.bval[i];
  }

  //---- < 2 > S P I 送 受 信 -----------------------------------------------
  //[2-1] SPI送受信を同時に行う
  if (slave.remained() == 0) {
    slave.queue(r_message_buf, s_message_buf, MSG_BUFF);
  }
  delayMicroseconds(50);

  //[2-2] SPI受信データバッファを別のバッファ配列に転写
  memcpy(r_spi_message_buf.bval, r_message_buf, MSG_BUFF);
  //delayMicroseconds(1);

  while (slave.available())
  {
    slave.pop();//end transaction

    // SPI受信配列からUDP送信配列にデータを転写
    for (int i = 0; i < MSG_BUFF; i++) {
      s_upd_message_buf.bval[i] = r_spi_message_buf.bval[i];
    }
  
  //---- < 3 > U D P 送 信 ----------------------------------------------
  //[3-1] UDP送信を実行
  sendUDP();
  }
  delayMicroseconds(100);
}
