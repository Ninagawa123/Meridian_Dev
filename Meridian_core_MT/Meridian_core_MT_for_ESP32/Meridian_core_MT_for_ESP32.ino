// Meridian_core_for_ESP32_MT_20220204 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Teensy4.0 - (SPI) - ESP32DevKitC - (Wifi/UDP) - PC/python
// ESP32用のマルチスレッド化したMeiridian_core
// PS4コントローラ対応修正済み

//[E-2] ライブラリ導入 -----------------------------------
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
#include <ESP32DMASPISlave.h>
#include <FastCRC.h>
ESP32DMASPI::Slave slave;
#include <PS4Controller.h>//PS4コントローラー
#include <ESP32Wiimote.h>
ESP32Wiimote wiimote;

//PS4用を新規接続するためのペアリング情報解除設定
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#define REMOVE_BONDED_DEVICES 1 // 0でバインドデバイス情報表示、1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)
#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

//[E-3] 各種設定 ３#DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号
#define JOYPAD_MOUNT 0 ////ジョイパッドの搭載 0:なし、1:Wii_yoko, 2:Wii+Nun, 3:PS3, 4:PS4, 5:WiiPRO, 6:Xbox
//#define JOYPAD_POLLING 10 ////ジョイパッドの問い合わせフレーム間隔(ms)

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
int checksum; //チェックサム計算用
long frame_count = 0;
long error_count_udp = 0;//UDP受信エラーカウント用
long error_count_spi = 0;//SPI受信エラーカウント用
uint8_t* s_spi_meridim_dma;//DMA用
uint8_t* r_spi_meridim_dma;//DMA用

//フラグ関連
bool udp_resv_flag = 1;//UDPスレッドでの受信反応フラグ
bool udp_send_flag = 0;//UDP送信完了フラグ
bool spi_resv_flag = 0;//SPI受信完了フラグ

//マルチスレッドのタスクハンドル格納用
TaskHandle_t thp[4];

//wifi設定
WiFiUDP udp;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE + 2];
  uint8_t bval[MSG_BUFF + 4];
} UnionData;
UnionData s_spi_meridim; //SPI受信用共用体
UnionData r_spi_meridim; //SPI受信用共用体
UnionData s_udp_meridim; //UDP送信用共用体
UnionData r_udp_meridim; //UDP受信用共用体
//UnionData r_dummy_meridim; //UDP受信用共用体 テスト用ダミー

//コントローラー用変数
unsigned short pad_btn = 0;
short pad_stick_R = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_stick_V = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;//ディスプレイ表示用（バイナリ表示の上位ビット）

void setup()
{
  Serial.begin(2000000);
  delay(130);//シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
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
  initBluetooth();
  Serial.print("ESP32's Bluetooth Mac Address is => ");
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));

  // BTペアリング情報
  int count = esp_bt_gap_get_bond_device_num();
  if (!count) {
    Serial.println("No bonded BT device found.");
  } else {
    Serial.print("Bonded BT device count: "); Serial.println(count);
    if (PAIR_MAX_DEVICES < count) {
      count = PAIR_MAX_DEVICES;
      Serial.print("Reset bonded device count: "); Serial.println(count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    if (ESP_OK == tError) {
      for (int i = 0; i < count; i++) {
        Serial.print("Found bonded BT device # "); Serial.print(i); Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
        if (REMOVE_BONDED_DEVICES) {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if (ESP_OK == tError) {
            Serial.print("Removed bonded BT device # ");
          } else {
            Serial.print("Failed to remove bonded BT device # ");
          }
          Serial.println(i);
        }
      }
    }
  }

  //コントローラの接続開始
  //PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4) {
    PS4.begin("e8:68:e7:30:b4:52");//ESP32のMACが入ります.PS4にも設定します。
    Serial.println("PS4 controller connecting...");
  }
  //Wiiコントローラの接続開始
  if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
    wiimote.init();
    wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
    Serial.println("Wiimote connecting...");
  }
  //delay(1000);

  //UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

  // DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定  ※+4は不具合対策
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  //※バッファサイズは4で割り切れる必要があり、なおかつ末尾に4バイト分0が入る不具合があるのでその対策

  // 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4);// ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);

  //初回の送信データを作成してセット
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    s_spi_meridim.sval[i] = 0;//配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
  }
  checksum = short(~checksum);//チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum); //データ末尾にチェックサムを入れる
  for (int i = 0; i < MSG_BUFF + 4; i++) { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
  }

  //ここでスレッドを立てる宣言（無線系はすべてCORE0で動くらしい メインループはCORE1）
  xTaskCreatePinnedToCore(Core0_UDP_s, "Core0_UDP_s", 4096, NULL, 7, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 8192, NULL, 5, &thp[1], 0);

  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て（1か2のみ)
  slave.setQueueSize(1); // キューサイズ　とりあえず1
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
}


//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// ■ Bluetoothペアリング設定用 -----
bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

// ■ Bluetoothペアリングアドレス取得用 -----
char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}


// ■ 受信用の関数 パケットが来ていれば、MSG_BUFF 分だけr_udp_meridim.bval配列に保存 -----
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
    udp_resv_flag = 1;
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

// ■ チェックサムの算出関数 ----------------------------------------------------------
short checksum_val(short arr[], int len) {
  int cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    cksm += int(arr[i]);
  }
  //cksm = short(~cksm);
  return ~cksm;
}

// ■ チェックサムの判定関数 ----------------------------------------------------------
bool checksum_rslt(short arr[], int len) {
  int cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    cksm += int(arr[i]);
  }
  if (short(~cksm) == arr[len - 1]) {
    return true;
  }
  return false;
}

//-------------------------------------------------------------------------
//---- U D P 送 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_s(void *args) {//サブCPU(Core0)で実行するプログラム
  delay(1000);
  while (1) {
    if (udp_send_flag == 1) {
      sendUDP();
      udp_send_flag = 0;
    }
    delay(2);//1/1000秒待つ
  }
}

// ■ PS4コントローラ受信用関数 ------------------------------------------------

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

    pad_stick_L = pad_stick_L_x * 256 + pad_stick_L_y;
    pad_stick_R = pad_stick_R_x * 256 + pad_stick_R_y;
    pad_stick_V = pad_L2_val * 256 + pad_R2_val;
  }
}

// ■ Wiiコントローラ(横持ち・横持ち+ヌンチャク)受信用関数 -----------------------------

void Wiipad_receive_h() {
  wiimote.task();
  if (wiimote.available() > 0) {
    uint16_t button = wiimote.getButtonState();
    NunchukState nunchuk = wiimote.getNunchukState();
    pad_btn = 0;

    if (button & 0x0400) pad_btn |= (B00000000 * 256) + B00100000; //right
    if (button & 0x0100) pad_btn |= (B00000000 * 256) + B01000000; //down
    if (button & 0x0200) pad_btn |= (B00000000 * 256) + B00010000; //up
    if (button & 0x0800) pad_btn |= (B00000000 * 256) + B10000000; //left

    if (button & 0x0008) pad_btn |= (B10000000 * 256) + B00000000; //A
    if (button & 0x0002) pad_btn |= (B01000000 * 256) + B00000000; //1
    if (button & 0x0001) pad_btn |= (B00100000 * 256) + B00000000; //2
    if (button & 0x0004) pad_btn |= (B00010000 * 256) + B00000000; //B

    if (button & 0x0010) pad_btn |= (B00000000 * 256) + B00000001; //+
    if (button & 0x1000) pad_btn |= (B00000000 * 256) + B00001000; //-

    if (button & 0x0080) pad_btn |= (B00000000 * 256) + B01010000; //same as up & down//home

    if (JOYPAD_MOUNT == 2) {
      NunchukState nunchuk = wiimote.getNunchukState();
      int calib_l1x = 5;//LスティックX軸のセンターのキャリブレーション値
      int calib_l1y = -6;//LスティックY軸のセンターのキャリブレーション値
      pad_stick_L = ((nunchuk.xStick+calib_l1x-127) * 256 + (nunchuk.yStick-127+calib_l1y));
      if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
    Serial.print(127-nunchuk.xStick);
    Serial.print(",");
    Serial.println(nunchuk.yStick-127);

    delay(2);//ここの数値でCPU負荷を軽減できるかも
  }
}

//-------------------------------------------------------------------------
//---- Bluetooth 用 ス レ ッ ド --------------------------------------------
//-------------------------------------------------------------------------

void Core0_BT_r(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    //コントローラの受信ループ
    if (JOYPAD_MOUNT == 4) {
      PS4pad_receive();
    }
    //Wiiコントローラの受信ループ
    if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
      Wiipad_receive_h();
    }
    delay(1);//JOYPAD_POLLING ms秒待つ
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
  // ● UDP受信データ「r_udp_meridim」に中身が入った状態

  //[1-2] UDP受信配列からSPI送信配列にデータを転写
  if (udp_resv_flag == 1) {

    // ● UDP受信データ「r_udp_meridim」のチェックサムを確認
    if (checksum_rslt(r_udp_meridim.sval, MSG_SIZE))
    {
      // ● 受信成功ならUDP受信データをSPI送信データに上書き更新する
      memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
      //ここでこのパートのエラーフラグも消す
      s_spi_meridim.bval[177] &= 0b10111111;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを下げる
      //Serial.println("UDPrvOK");
    } else {
      // ● 受信失敗ならUDP受信データをSPI送信データに上書き更新せず、前回のSPI送信データにエラーフラグだけ上乗せする
      s_spi_meridim.bval[177] |= 0b01000000;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを上げる
      error_count_udp ++;
      Serial.print("U:"); Serial.println(error_count_udp);//    Serial.print("/");    Serial.println(frame_count);
    }
    udp_resv_flag = 0;
  }

  // ●「s_spi_meridim」にはチェックサム済みの「r_udp_meridim」が転記され、ESP32UDP受信エラーフラグも入った状態

  //---- < 2 > S P I 送 信 信 号 作 成  -----------------------------------------------

  //[2-1] ユーザー定義の送信データの書き込み
  // Teensyへ送るデータをこのパートで作成、書き込み

  //[2-2] リモコンデータの書き込み
  s_spi_meridim.sval[80] = pad_btn;
  //if (s_spi_meridim.sval[80] != 0) {//ボタンのモニタリング用
  //  Serial.println(s_spi_meridim.sval[80], BIN);
  //}
  s_spi_meridim.sval[81] = pad_stick_L;
  s_spi_meridim.sval[82] = pad_stick_R;
  s_spi_meridim.sval[83] = pad_stick_V;

  // ● SPI送信データ「s_spi_meridim」はESP32で受けたリモコンデータが上書きされた状態

  //[2-3] チェックサムの追記
  // ● SPI送信データ「s_spi_meridim」にチェックサムを追記
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);

  // ● SPI送信データ「s_spi_meridim」はチェックサムも入り完成している

  //---- < 3 > S P I 送 受 信 -----------------------------------------------
  while (slave.available())//完了した（受信した）トランザクション結果のサイズ
  {

    // ↓↓↓↓ SPI送受信用のデータはここで処理しなくてはならない ↓↓↓↓

    //[3-1] 完成したSPI送信データをDMAに転記
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);

    // ↑↑↑↑ SPI送受信用のデータはここまでで処理しなくてはならない ↑↑↑↑

    slave.pop();//DMAのデータ配列の先頭を削除
  }

  //[3-2] SPI送受信の実行
  //SPIの送受信:キューが送信済みであればセットされた送信データを送信する。
  if (slave.remained() == 0) {//キューに入れられた（完了していない）トランザクションのサイズが0なら実行
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    spi_resv_flag = 1;//SPI受信完了フラグを上げる
    frame_count = frame_count + 1;//SPIの受信をもってフレームカウントとする
  }

  //---- < 4 > U D P 送 信 信 号 作 成 ----------------------------------------
  if (spi_resv_flag == 1) {//SPI受信完了フラグが立っていればUDP送信スレッドにフラグで知らせる

    //[4-1] 受信したSPI送信データをUDP送信データに転記
    memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4);
    // ● UDP送信データ"s_udp_meridim"に中身が入った状態

    //---- < 5 > U D P 送 信 ----------------------------------------------
    //[5-1] UDP送信を実行（フラグを立ててスレッドに知らせる）
    udp_send_flag = 1;//UDP送信準備完了フラグを上げる
    spi_resv_flag = 0;//SPI受信完了フラグを下げる
  }
  delayMicroseconds(1);
}
