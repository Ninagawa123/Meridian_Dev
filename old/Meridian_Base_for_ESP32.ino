//Meridian_211118_for_ESP32
//ESP32_DevKitC

/*
  -------------------------------------------------------------------------
  ---- ESP32 ピンアサイン [E-1-0] --------------------------------------
  -------------------------------------------------------------------------
  [3V3]               -> GND
  [EN]                -> ICS_3rd_TX
  [VP,36]             -> ICS_3rd_RX
  [VN,39]             -> LED（lights up when the processing time is not within the specified time.）
  [34]                -> (NeoPixel Data)
  [35]                -> (NeoPixel Clock)
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
  [5V]                ->

  [GND]               ->
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


//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
ESP32DMASPI::Slave slave;
long spi_ok = 0;//通信のエラー数
long spi_trial = 0;//通信回数
long udp_ok = 0;//通信のエラー数
long udp_trial = 0;//通信回数

uint8_t* s_message_buf;
uint8_t* r_message_buf;
int checksum;

//wifi設定
const char* ssid = "xxxxxx"; //アクセスポイントのSSID
const char* password = "xxxxxx"; //アクセスポイントのパスワード
const char* send_address = "192.168.xxx.xxx"; //送り先のIPアドレス
const int send_port = 22222;  //送り先のポート番号
const int receive_port = 22224;  //このESP32のポート番号

WiFiUDP udp;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_upd_message_buf; //送信用共用体のインスタンスを宣言
UnionData r_upd_message_buf; //受信用共用体のインスタンスを宣言
UnionData s_message_buf_2;
UnionData r_message_buf_2;
UnionData s_message_buf_3;


//-------------------------------------------------------------------------
//---- 各 種 モ ー ド 設 定 0:OFF, 1:ON [S-4] -------------------------------
//-------------------------------------------------------------------------
bool monitor_udp_send = 0; //ESP32でのシリアル表示:UDP送信データ
bool monitor_udp_resv = 0; //ESP32でのシリアル表示:UDP受信データ
bool monitor_spi_send = 0; //ESP32でのシリアル表示:SPI送信データ
bool monitor_spi_resv = 0; //ESP32でのシリアル表示:SPI受信データ
bool monitor_udp_resv_check = 0; //ESP32でのシリアル表示:UDP受信成功の可否
bool monitor_udp_resv_error = 1; //ESP32でのシリアル表示:UDP受信エラー率
bool monitor_spi_resv_check = 0; //ESP32でのシリアル表示:SPI受信成功の可否
bool monitor_spi_resv_error = 1; //ESP32でのシリアル表示:SPI受信エラー率
bool monitor_all_error = 0; //ESP32でのシリアル表示:全経路の受信エラー率

void setup()
{
  Serial.begin(2000000);
  delay(500);//シリアル準備待ち用ディレイ
  Serial.println("Serial Start...");

  //WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  Serial.println("Connecting to WiFi to : " + String(ssid));//接続先を表示
  delay(100);
  WiFi.begin(ssid, password);//Wifiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(100);//接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected.");//WiFi接続完了通知
  Serial.print("WiFi connected. ESP32's IP address is : ");
  Serial.println(WiFi.localIP());//デバイスのIPアドレスの表示

  //UDP通信の開始
  udp.begin(receive_port);
  delay(500);

  //SPI通信用のDMA（大容量通信用メモリバッファ）の設定
  s_message_buf = slave.allocDMABuffer(MSG_BUFF);
  r_message_buf = slave.allocDMABuffer(MSG_BUFF);

  // バッファのリセット
  memset(s_message_buf, 0, MSG_BUFF);
  memset(r_message_buf, 0, MSG_BUFF);

  // 初回送信データの作成
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) // 配列の末尾以外を加算していく
  {
    s_message_buf[i] = short(MSG_SIZE - i);
    checksum += MSG_SIZE - i;
  }
  s_message_buf[MSG_SIZE - 1] = short(checksum & 0xffff ^ 0xffff); //末尾にチェックサムを代入

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
    udp_trial ++;

    //データのシリアルモニタ表示
    if (monitor_udp_resv == 1) {
      Serial.print("[UDP RESV] ");
      for (int i = 0; i < MSG_SIZE; i++)
      {
        Serial.print(String(r_upd_message_buf.sval[i]));
        Serial.print(", ");
      }
      Serial.println();
    }

    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i++) {
      checksum += r_upd_message_buf.sval[i];
    }
    checksum = ~checksum & 0xffff;
    
    if (short(r_upd_message_buf.sval[MSG_SIZE - 1]) == short(checksum)) {
      if (monitor_udp_resv_check == 1) {
        Serial.print("[UDP CKsm] OK!:  "); Serial.print(short(checksum));
        Serial.print(" : "); Serial.println(short(r_upd_message_buf.sval[MSG_SIZE - 1]));
      }
      udp_ok ++;
    } else
    {
      if (monitor_udp_resv_check == 1) {
        Serial.print("[UDP CKsm]*ERR*: "); Serial.print(short(checksum));
        Serial.print(" : "); Serial.println(short(r_upd_message_buf.sval[MSG_SIZE - 1]));
      }
    }

    if (udp_trial >= 1000) { //エラー率の表示
      if (monitor_udp_resv_error == 1) {
        Serial.print("[UDP error rate] ");
        Serial.print(float(udp_trial - udp_ok) / float(udp_trial) * 100);
        Serial.print(" %  ");
        Serial.print(udp_trial - udp_ok);
        Serial.print("/");
        Serial.println(udp_trial);
      }
    }

    if (udp_trial >= 30000) {
      udp_trial = 0;
      udp_ok = 0;
    }
  } else
  {
    if ((monitor_udp_resv == 1) or (monitor_udp_resv_check == 1)) {
      Serial.println("waiting UDP...");
    }
  }
}

// ■ 送信用の関数 ----------------------------------------------------------
void sendUDP() {
  String test = "";//表示用の変数

  udp.beginPacket(send_address, send_port);//UDPパケットの開始

  for (int i = 0; i < MSG_BUFF; i++) {
    udp.write(s_upd_message_buf.bval[i]);//１バイトずつ送信
    if (i % 2 == 0) {//表示用に送信データを共用体から2バイトずつ取得
      test += String(s_upd_message_buf.sval[i / 2]) + ", ";
    }
  }
  //送信データの表示
  if (monitor_udp_send == 1) {
    Serial.println("[UDP SEND] " + test );//送信データ（short型）を表示
  }
  udp.endPacket();//UDPパケットの終了
}


//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  Serial.println();

  //UDP受信。もしデータパケットが来ていれば受信する
  receiveUDP();

  //UDP受信配列からSPI送信配列にデータを転写
  for (int i = 0; i < MSG_BUFF; i++) {
    s_message_buf[i] = r_upd_message_buf.bval[i];
    //s_message_buf_3.bval[i] = r_upd_message_buf.bval[i];
  }

  // SPI send data
  if (slave.remained() == 0) {
    slave.queue(r_message_buf, s_message_buf, MSG_BUFF);
  }
  delayMicroseconds(50);

  memcpy(r_message_buf_2.bval, r_message_buf, MSG_BUFF);
  //delayMicroseconds(1);

  while (slave.available())
  {

    // show send data
    if (monitor_spi_send == 1) {
      Serial.print("[SPI SEND] : ");
      for (int i = 0; i < MSG_SIZE; i++)
      {
        Serial.print(s_message_buf[i]);
        Serial.print(",");
      }
      Serial.println();
    }

    // show received data
    if (monitor_spi_resv == 1) {
      Serial.print("[SPI RESV] : ");
      for (size_t i = 0; i < MSG_SIZE; ++i) {
        Serial.print(r_message_buf_2.sval[i]);
        Serial.print(",");
      }
      Serial.println();
    }

    slave.pop();//end transaction??

    // SPI受信のチェックサムを計算
    spi_trial ++;
    int checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1 ; i++) {
      checksum += int(r_message_buf_2.sval[i]);
    }
    checksum = (checksum ^ 0xffff) & 0xffff;
    Serial.print("spi rsvd calclated:");
    Serial.println(checksum);

    // SPI受信のチェックサム結果を判定・表示
    if (short(checksum) == short(r_message_buf_2.sval[MSG_SIZE - 1])) {
      if (monitor_spi_resv_check == 1) {
        Serial.print("[SPI CKsm] OK!:  "); Serial.print(short(checksum));
        Serial.print(" : "); Serial.println(short(r_message_buf_2.sval[MSG_SIZE - 1]));
      }
      spi_ok ++;
    } else {
      if (monitor_spi_resv_check == 1) {
        Serial.print("[SPI CKsm]*ERR*: "); Serial.print(short(checksum));
        Serial.print(" : "); Serial.println(short(r_message_buf_2.sval[MSG_SIZE - 1]));
      }
    }
  }

  if (spi_trial >= 500) { //エラー率の表示
    if (monitor_spi_resv_error == 1) {
      Serial.print("[SPI error rate] ");
      Serial.print(float(spi_trial - spi_ok) / float(spi_trial) * 100);
      Serial.print(" %  ");
      Serial.print(spi_trial - spi_ok);
      Serial.print("/");
      Serial.println(spi_trial);
    }
  }
  if (spi_trial >= 30000) {
    spi_trial = 0;
    spi_ok = 0;
  }

  // SPI受信配列からUDP送信配列にデータを転写
  for (int i = 0; i < MSG_BUFF; i++) {
    s_upd_message_buf.bval[i] = r_message_buf_2.bval[i];
  }

  sendUDP();//UDPで送信

  //全経路の受信エラー率表示(※未実装)
  if (monitor_all_error == 1) {
    if  (s_message_buf_3.sval[84] != 0) {
      Serial.print("[UDP_resv_err] ");
      Serial.print(s_message_buf_3.sval[85]);
      Serial.print("/");
      Serial.print(s_message_buf_3.sval[84]);
      Serial.print(" ");
      Serial.print(float(s_message_buf_3.sval[85]) / (float(s_message_buf_3.sval[84])) * 100);
      Serial.println(" %");
    }
        if  (s_message_buf_3.sval[82] != 0) {
      Serial.print("[SPI_resv_err] ");
      Serial.print(s_message_buf_3.sval[83]);
      Serial.print("/");
      Serial.print(s_message_buf_3.sval[82]);
      Serial.print(" ");
      Serial.print(float(s_message_buf_3.sval[83]) / (float(s_message_buf_3.sval[82])) * 100);
      Serial.println(" %");
    }
    delayMicroseconds(100);
  }
}
