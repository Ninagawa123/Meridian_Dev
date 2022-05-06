
// Meridian_core_MT_for_ESP32_2022.05.06d By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Teensy4.0 - (SPI) - ESP32DevKitC - (Wifi/UDP) - PC/python
// ESP32用のマルチスレッド化したMeiridian_core
// PS4コントローラ再対応
// UDP送受、UDP送信、両方ともスレッド化し、フラグによりデータの流れを円滑化
// ただしBTを同時接続するとエラーでデータが欠損するので、リモコンは一旦わすれる。
// リモコンの運用は今後、PC-UDP接続時はPC側でリモコン処理、
// PCレスのスタンドアロン時はESP32でBTを使い、Wiiをキャンセル（未導入）で対応する。
// IP固定化の要素を追加。


//---------------------------------------------------
// [SETTING] 各種設定 (ES-1) -------------------------
//---------------------------------------------------

// (ES-1-1) 変更頻度高め
#define VERSION "Meridian_core_MT_for_ESP32_2022.05.06d"//バージョン表示
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.26" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
//#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
//#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
//#define SEND_IP "192.168.xx.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）

//IPを固定する場合は下記の4項目を設定し、(ES-4-1) WiFi.configを有効にする 固定しない場合はコメントアウト必須
//IPAddress ip(192,168,xx,xx);//ESP32のIPアドレスを固定する場合のアドレス
//IPAddress subnet(255,255,255,0);//ESP32のIPアドレス固定する場合のサブネット
//IPAddress gateway(192,168,xx,xx);//ルーターのゲートウェイを入れる
//IPAddress DNS(8,8,8,8);//DNSサーバーの設定（使用せず）

// (ES-1-2) シリアルモニタリング切り替え
//-特になし-

// (ES-1-3) マウント有無とピンアサイン
//-特になし-

// (ES-1-4) 各種初期設定
#define JOYPAD_MOUNT 0 //ジョイパッドの搭載 0:なし、1:Wii_yoko, 2:Wii+Nun, 3:PS3, 4:PS4, 5:WiiPRO, 6:Xbox
//#define JOYPAD_POLLING 10 ////ジョイパッドの問い合わせフレーム間隔(ms)
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define SERIAL_PC 2000000 //ESP-PC間のシリアル速度（モニタリング表示用）

// (ES-1-5) その他固定値
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号
#define REMOVE_BONDED_DEVICES 0 //0でバインドデバイス情報表示、1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)
#define PAIR_MAX_DEVICES 20 //接続デバイスの記憶可能数


//---------------------------------------------------
// [LIBRARY] ライブラリ関連 (ES-2-LIB) ----------------
//---------------------------------------------------

// (ES-2-1) ライブラリ全般
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
WiFiUDP udp;//wifi設定
#include <ESP32DMASPISlave.h>
ESP32DMASPI::Slave slave;
#include <PS4Controller.h>//PS4コントローラー
#include <ESP32Wiimote.h>//Wiiコントローラー
ESP32Wiimote wiimote;
//#include <FastCRC.h>

// (ES-2-2) PS4用を新規接続するためのペアリング情報解除設定
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

//---------------------------------------------------
// [VARIABLE] 変数関連 (ES-3-VAL) --------------------
//---------------------------------------------------

// (ES-3-1) 変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
int checksum; //チェックサム計算用
long frame_count = 0;
long error_count_udp = 0;//UDP受信エラーカウント用
long error_count_spi = 0;//SPI受信エラーカウント用
long error_count_esp_skip = 0;//ESPのPCからの受信連番カウントエラー用
uint8_t* s_spi_meridim_dma;//DMA用
uint8_t* r_spi_meridim_dma;//DMA用
TaskHandle_t thp[4];//マルチスレッドのタスクハンドル格納用

// (ES-3-2) フラグ関連
bool udp_revd_flag = 0;//UDPスレッドでの受信完了フラグ
bool udp_resv_process_queue_flag = 0;//UDP受信済みデータの処理待ちフラグ
bool udp_send_flag = 0;//UDP送信完了フラグ
bool spi_revd_flag = 0;//SPI受信完了フラグ
bool udp_receiving_flag = 0;//UDPスレッドでの受信中フラグ（送信抑制）
bool udp_sending_flag = 0;//UDPスレッドでの送信中フラグ（受信抑制）
char frame_sync_s = 0;//フレーム毎に0-199をカウントし、送信用Meridm[88]の下位8ビットに格納
char frame_sync_r_expect = 0;//フレーム毎に前回受信値に+１として受信値と比較（0-199)
char frame_sync_r_resv = 0;//今フレームに受信したframe_sync_rを格納

// (ES-3-3) 共用体の設定. 共用体はたとえばデータをショートで格納し,バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE + 2];
  uint8_t bval[MSG_BUFF + 4];
} UnionData;
UnionData s_spi_meridim; //SPI受信用共用体
UnionData r_spi_meridim; //SPI受信用共用体
UnionData s_udp_meridim; //UDP送信用共用体
UnionData r_udp_meridim; //UDP受信用共用体

// (ES-3-4) コントローラー用変数
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
  //-------------------------------------------------------------------------
  //---- (ES-4) 起動時設定 --------------------------------------------------
  //-------------------------------------------------------------------------

  // (ES-4-1) WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  //WiFi.config(ip, gateway, subnet, DNS);//※IPを固定にする場合はコメントアウトをはずして有効にする(ES-1-1)も変更すること
  WiFi.begin(AP_SSID, AP_PASS);//WiFiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50);//接続が完了するまでループで待つ
  }

  // (ES-4-2) シリアル設定
  Serial.begin(SERIAL_PC);
  delay(130);//シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println("Serial Start...");

  // (ES-4-3) シリアル表示
  Serial.println(VERSION); //バージョン表示
  Serial.println("Connecting to WiFi to : " + String(AP_SSID));//接続先を表示
  Serial.println("WiFi connected.");//WiFi接続完了通知
  Serial.print("ESP32's IP address is  => ");//ESP32自身のIPアドレスの表示
  Serial.println(WiFi.localIP());
  Serial.print("Set PC's IP address    => ");//接続先PCのIPアドレスの表示
  Serial.println(SEND_IP);
  initBluetooth();
  Serial.print("ESP32's Bluetooth Mac Address is => ");//ESP32自身のBluetoothMacアドレスを表示
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));

  // (ES-4-4) BTペアリング情報
  int bt_count = esp_bt_gap_get_bond_device_num();
  if (!bt_count) {
    Serial.println("No bonded BT device found.");
  } else {
    Serial.print("Bonded BT device count: "); Serial.println(bt_count);
    if (PAIR_MAX_DEVICES < bt_count) {
      bt_count = PAIR_MAX_DEVICES;
      Serial.print("Reset bonded device count: "); Serial.println(bt_count);
    }
    esp_err_t tError =  esp_bt_gap_get_bond_device_list(&bt_count, pairedDeviceBtAddr);
    if (ESP_OK == tError) {
      for (int i = 0; i < bt_count; i++) {
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

  // (ES-4-5) コントローラの接続開始
  // (ES-4-5-1) PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4) {
    PS4.begin("e8:68:e7:30:b4:52");//ESP32のMACが入ります.PS4にも設定します。
    Serial.println("PS4 controller connecting...");
  }

  // (ES-4-5-2) Wiiコントローラの接続開始
  if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
    wiimote.init();
    wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
    Serial.println("Wiimote connecting...");
  }

  // (ES-4-6) UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

  // (ES-4-7) DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  //※バッファサイズは4で割り切れる必要があり、なおかつ末尾に4バイト分0が入る不具合があるのでその対策

  // (ES-4-8) 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4);// ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);

  // (ES-4-9) 初回の送信データを作成してセット
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

  // (ES-4-10) マルチスレッドの宣言（無線系はすべてCORE0で動くとのこと.メインループはCORE1）
  //xTaskCreatePinnedToCore(Core0_UDP_s, "Core0_UDP_s", 4096, NULL, 10, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0_UDP_r, "Core0_UDP_r", 4096, NULL, 20, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);

  // (ES-4-11) SPI通信の設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
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
//---- U D P 受 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_r(void *args) {//サブCPU(Core0)で実行するプログラム
  delay(1000);
  while (1) {
    while (udp_sending_flag == false) {
      receiveUDP();//常にUDPの受信を待ち受けし、受信が完了したらフラグを挙げる
      delay(1);//1/1000秒待つ
    }
    delay(1);//1/1000秒待つ
  }
}

// ■ 受信用の関数 パケットが来ていれば、MSG_BUFF 分だけr_udp_meridim.bval配列に保存 -----
void receiveUDP() {
  //Serial.println("receiveUDP()");
  while (udp_sending_flag) {
    delayMicroseconds(10);
  }
  int packetSize = udp.parsePacket();//受信済みのパケットサイズを取得
  byte tmpbuf[MSG_BUFF];

  //データの受信
  if (packetSize >= MSG_BUFF) {//受信済みのパケットサイズを確認し、配列分受信できていたら読み出し


    udp_receiving_flag = 1;
    udp.read(tmpbuf, MSG_BUFF);
    memcpy(r_udp_meridim.bval, tmpbuf, MSG_BUFF);
    //for (int i = 0; i < MSG_BUFF; i++)
    //{
    //  r_udp_meridim.bval[i] = tmpbuf[i];
    //}

    //delayMicroseconds(10);
    udp_receiving_flag = false;
    udp_revd_flag = true;//r_udp_meridimに受信完了した旨のフラグを挙げる
  }
}


//-------------------------------------------------------------------------
//---- U D P 送 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_s(void *args) {//サブCPU(Core0)で実行するプログラム
  delay(1000);
  while (1) {
    if (udp_send_flag) {//メインループにより送信フラグが挙がったら送信実行
      udp_sending_flag = true;//送信busyフラグを挙げる.
      sendUDP();
      delayMicroseconds(10);
      udp_sending_flag = false;//送信busyフラグを下げる.
      udp_send_flag = false;
    }
    delay(1);//1/1000秒待つ
  }
}

// ■ 送信用の関数 ----------------------------------------------------------
void sendUDP() {
  udp.beginPacket(SEND_IP, SEND_PORT);//UDPパケットの開始
  udp.write(s_udp_meridim.bval, MSG_BUFF);//一括送信
  udp.endPacket();//UDPパケットの終了
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
      pad_stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
      if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
    Serial.print(127 - nunchuk.xStick);
    Serial.print(",");
    Serial.println(nunchuk.yStick - 127);

    delay(1);//ここの数値でCPU負荷を軽減できるかも
  }
}


//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //----  [ 1 ]  U D P 受 信  --------------------------------------------------

  //[1-1] UDP受信の実行
  //  --->> [check!] UDP受信は別スレッド void Core0_UDP_r() で実行されている.

  //[1-2] UDP受信配列からSPI送信配列にデータを転写.
  if (udp_revd_flag) {//UDP受信完了フラグをチェック.(UDP受信データ r_udp_meridim の更新が完了している状態か?)
    udp_revd_flag = false;//UDP受信完了フラグを下げる.

    // [1-2-1] UDP受信データ r_udp_meridim のチェックサムを確認.
    if (checksum_rslt(r_udp_meridim.sval, MSG_SIZE))
    {
      // [1-2-1-a-1] 受信成功ならUDP受信データをSPI送信データに上書き更新する.
      memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
      delayMicroseconds(1);

      // このパートのエラーフラグを下げる
      s_spi_meridim.bval[177] &= 0B10111111;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを下げる.

      // [1-2-1-a-2] 通信エラー処理(スキップ検出)
      frame_sync_r_resv = s_spi_meridim.bval[176];//数値を受け取る

      //予測値のカウントアップ
      frame_sync_r_expect ++;
      if (frame_sync_r_expect > 199) {
        frame_sync_r_expect = 0;
      }

      //カウント受信の比較
      //Serial.print(int(frame_sync_r_expect));
      //Serial.print(":");
      //Serial.println(int(frame_sync_r_resv));

      if (frame_sync_r_resv == frame_sync_r_expect)//予想通りなら取りこぼしなし
      {
        s_spi_meridim.bval[177] &= B11111011;//エラーフラグ10番(ESPのPCからのUDP取りこぼし検出)をオフ
      } else
      {
        frame_sync_r_expect = frame_sync_r_resv;//受信値を正解の予測値だったとする
        s_spi_meridim.bval[177] |= B00000100; //エラーフラグ10番(ESPのPCからのUDP取りこぼし検出)をオン
        error_count_esp_skip ++;
      }

      //UDPのスキップ回数を表示
      //Serial.print("  count ");
      //Serial.println(int(error_count_esp_skip));

      //受信カウントをそのまま送信カウントとすることで、PCデータがシーケンシャルにTsyに届いてるかのチェックとする
      frame_sync_s = frame_sync_r_resv;

    } else {
      // [1-2-1-b] 受信失敗ならUDP受信データをSPI送信データに上書き更新せず, 前回のSPI送信データにエラーフラグだけ上乗せする.
      s_spi_meridim.bval[177] |= 0B01000000;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを上げる.
      error_count_udp ++;//エラーカウンタをアップ.
    }
  }
  //  --->> [check!] ここで s_spi_meridim にはチェックサム済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.


  //----  [ 2 ]  S P I 送 信 信 号 作 成  -----------------------------------------------

  //[2-1] ユーザー定義の送信データの書き込み
  //Teensyへ送るデータをこのパートで作成, 書き込み.
  //  --->> [check!] 現在はここでとくに何もしない.

  //[2-2] リモコンデータの書き込み
  s_spi_meridim.sval[80] = pad_btn;
  s_spi_meridim.sval[81] = pad_stick_L;
  s_spi_meridim.sval[82] = pad_stick_R;
  s_spi_meridim.sval[83] = pad_stick_V;
  //  --->> [check!] ここでSPI送信データ s_spi_meridim はESP32で受けたリモコンデータが上書きされた状態.

  //[2-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）
  s_spi_meridim.bval[176] = short(frame_sync_s);

  //[2-4] チェックサムの追記
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);
  //  --->> [check!] ここでSPI送信データ s_spi_meridim はチェックサムが入り完成している状態.


  //----  [ 3 ]  S P I 送 受 信  -----------------------------------------------
  while (slave.available())//完了した（受信した）トランザクション結果のサイズ
  {

    // ↓↓↓↓ SPI送受信用のデータはここで処理しなくてはならない ↓↓↓↓
    //[3-1] 完成したSPI送信データをDMAに転記
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
    delayMicroseconds(1);
    //Serial.println("[3] copied to SPI_dma");
    // ↑↑↑↑ SPI送受信用のデータはここまでで処理しなくてはならない ↑↑↑↑

    slave.pop();//DMAのデータ配列の先頭を削除

  }

  //[3-2] SPI送受信の実行
  //SPIの送受信:キューが送信済みであればセットされた送信データを送信する.
  if (slave.remained() == 0) {//キューに入れられた（完了していない）トランザクションのサイズが0なら実行.
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    frame_count = frame_count + 1;//SPIの受信をもってフレームカウントとする

    //----  [ 4 ]  U D P 送 信 信 号 作 成 ----------------------------------------

    //[4-1] 受信したSPI送信データをUDP送信データに転記.
    memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4);
    delayMicroseconds(1);
    //  --->> [check!] UDP送信データ"s_udp_meridim"に中身が入った状態.


    //----  [ 5 ]  U D P 送 信 ----------------------------------------------
    //[5-1] UDP送信を実行
    udp_sending_flag = true;
    sendUDP();
    udp_sending_flag = false;
    /*
      while (udp_receiving_flag) //UDPが受信処理中なら送信しない
      {
      delayMicroseconds(2);
      }
    */
    udp_send_flag = true;//UDP送信準備完了フラグを挙げる.(スレッドCore0_UDP_sで送信実行)
  }
}
