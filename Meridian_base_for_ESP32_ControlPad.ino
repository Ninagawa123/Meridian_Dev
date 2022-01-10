//Meridian_base_for_ESP32_ControlPad_220111
//ESP32_DevKitC

//現状はPS4コントローラ対応に特化したバージョンです
//
//ライブラリと接続接続方法
//
//https://github.com/aed3/PS4-esp32
//1 上記を参考にライブラリを入れる
//2 ESP32/Meridian_core_for_ESP32_PassThroughでESP32のMACアドレスを調べる（下記起動時にシリアルモニタで確認可能）
//　 https://github.com/Ninagawa123/Meridian_core/tree/main/ESP32/Meridian_core_for_ESP32_PassThrough
//3 sixaxispairer でPS4コントローラに上記で調べたESP32のMACアドレスを書き込む
//4 166行目にESP32自身のMACアドレスを記入する。
//5 ESP32にスクリプトを書き込んでテスト。シリアル速度は2000000。
//6 ROS版のrosnode_meridim_demo_dpg.pyがGAMEPADデータの表示に簡易対応しています。

/*
  -------------------------------------------------------------------------
  ---- ESP32 ピンアサイン [E-1-0] --------------------------------------
  -------------------------------------------------------------------------
  [3V3]               ->
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
#include <PS4Controller.h>//PS4コントローラー
#include <ESP32DMASPISlave.h>
ESP32DMASPI::Slave slave;

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx" //送り先のIPアドレス
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号
#define JOYPAD_MOUNT 4 ////ジョイパッドの搭載 0:なし、1:Wii, 2:WiiPRO, 3:PS3, 4:PS4, 5:Xbox

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
uint8_t* s_message_buf;
uint8_t* r_message_buf;
int checksum; //チェックサム計算用

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

//コントローラー用変数
unsigned short pad_btn = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;

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
  Serial.print("ESP32's IP address is  => ");
  Serial.println(WiFi.localIP());//ESP32自身のIPアドレスの表示
  //ESP32自身のBluetoothMacアドレスを表示
  uint8_t bt_mac[6];
  String self_mac_address = "";
  String macadressforps4 = "";

  esp_read_mac(bt_mac, ESP_MAC_BT);
  //self_mac_address=String(bt_mac[0], HEX)+":"+String(bt_mac[1], HEX)+":"+String(bt_mac[2], HEX)+":"+String(bt_mac[3], HEX)+":"+String(bt_mac[4], HEX)+":"+String(bt_mac[5], HEX);
  Serial.printf("ESP32's Bluetooth Mac Address is => %02X:%02X:%02X:%02X:%02X:%02X\r\n", bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4], bt_mac[5]);
  //Serial.println(self_mac_address);

  //PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4) {
    PS4.begin("XX:XX:XX:XX:XX:XX");//ESP32のMACが入ります
  }

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


// ■ PS4コントローラ受信用関数 ----------------------------------------------------------

void PS4pad_receive() {
  // Below has all accessible outputs from the controller
  if (PS4.isConnected()) {
    pad_btn = 0;
    if (PS4.Right())    pad_btn |= (B00000000 * 256) + B00100000;
    if (PS4.Down())     pad_btn |= (B00000000 * 256) + B01000000;
    if (PS4.Up())       pad_btn |= (B00000000 * 256) + B00010000;
    if (PS4.Left())     pad_btn |= (B00000000 * 256) + B10000000;

    if (PS4.Square())   pad_btn |= (B10000000 * 256) + B00000000;
    if (PS4.Cross())    pad_btn |= (B01000000 * 256) + B00000000;
    if (PS4.Circle())   pad_btn |= (B00100000 * 256) + B00000000;
    if (PS4.Triangle()) pad_btn |= (B00010000 * 256) + B00000000;

    if (PS4.UpRight())  pad_btn |= (B00000000 * 256) + B00110000;
    if (PS4.DownRight())pad_btn |= (B00000000 * 256) + B01100000;
    if (PS4.UpLeft())   pad_btn |= (B00000000 * 256) + B10010000;
    if (PS4.DownLeft()) pad_btn |= (B00000000 * 256) + B11000000;

    if (PS4.L1())       pad_btn |= (B00000100 * 256) + B00000000;
    if (PS4.R1())       pad_btn |= (B00001000 * 256) + B00000000;

    if (PS4.Share())    pad_btn |= (B00000000 * 256) + B00000001;
    if (PS4.Options())  pad_btn |= (B00000000 * 256) + B00001000;
    if (PS4.L3())       pad_btn |= (B00000000 * 256) + B00000100;
    if (PS4.R3())       pad_btn |= (B00000000 * 256) + B00000010;

    if (PS4.PSButton()) pad_btn |= (B00000000 * 256) + B01010000;//same as up & down
    if (PS4.Touchpad()) pad_btn |= (B00000000 * 256) + B00101000;//same as left & right

    if (PS4.L2()) {
      pad_btn |= (0x00000001 * 256) + B00000000;
      pad_L2_val = constrain(PS4.L2Value(), 0, 255);
    }
    if (PS4.R2()) {
      pad_btn |= 512;//(0x00000010 * 256) + B00000000;
      pad_R2_val = constrain(PS4.R2Value(), 0, 255);
    }

    if (PS4.LStickX()) {
      pad_stick_L_x = constrain(PS4.LStickX(), -127, 127);
    }
    if (PS4.LStickY()) {
      pad_stick_L_y = constrain(PS4.LStickY(), -127, 127);
    }
    if (PS4.RStickX()) {
      pad_stick_R_x = constrain(PS4.RStickX(), -127, 127);
    }
    if (PS4.RStickY()) {
      pad_stick_R_y = constrain(PS4.RStickY(), -127, 127);
    }

    //if (PS4.Charging()) Serial.println("The controller is charging");
    //if (PS4.Audio()) Serial.println("The controller has headphones attached");
    //if (PS4.Mic()) Serial.println("The controller has a mic attached");
    //Serial.printf("Battery Level : %d\n", PS4.Battery());

    s_upd_message_buf.sval[80] = pad_btn;
    s_upd_message_buf.sval[81] = pad_stick_L_x * 256 + pad_stick_L_y;
    s_upd_message_buf.sval[82] = pad_stick_R_x * 256 + pad_stick_R_y;
    s_upd_message_buf.sval[83] = pad_L2_val * 256 + pad_R2_val;

    /*
      pad_btn_disp = pad_btn + 65536;
      Serial.print(pad_btn_disp, BIN);
      Serial.print(", ");
      Serial.print(pad_stick_L_x);
      Serial.print(", ");
      Serial.print(pad_stick_L_y);
      Serial.print(", ");
      Serial.print(pad_stick_R_x);
      Serial.print(", ");
      Serial.print(pad_stick_R_y);
      Serial.print(", ");
      Serial.print(pad_L2_val);
      Serial.print(", ");
      Serial.println(pad_R2_val);
      delay(10);
    */
  }
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


    //---- < 3 > 受信データの加工 -----------------------------------------------
    // SPI受信配列からUDP送信配列にデータを転写
    for (int i = 0; i < MSG_BUFF; i++) {
      s_upd_message_buf.bval[i] = r_spi_message_buf.bval[i];
    }

    //コントローラの受信
    if (JOYPAD_MOUNT == 4) {
      PS4pad_receive();
    }

    // 受信チェックサムの計算
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i++) {
      checksum += s_upd_message_buf.sval[i];
    }
    checksum = ~checksum & 0xffff;

    if ((short)s_upd_message_buf.sval[MSG_SIZE - 1] == (short)checksum)//チェックがOKならバッファから受信配列に転記
    {
      // 送信データを加工(リモコン受信データの差し込みなど)
      s_upd_message_buf.sval[49] = 10000;//ダミーデータ
    }
    else
    {
      s_upd_message_buf.sval[49] = -10000;//エラーメッセージを何かしら格納。これはダミーデータ
    }

    // チェックサムを計算して格納
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i ++) {
      checksum += s_upd_message_buf.sval[i];
    }
    checksum = ~checksum & 0xffff;
    s_upd_message_buf.sval[MSG_SIZE - 1] = checksum;

    //---- < 4 > U D P 送 信 ----------------------------------------------
    //[4-1] UDP送信を実行
    sendUDP();
  }
  delayMicroseconds(100);
}
