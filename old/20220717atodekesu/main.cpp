
// Meridian_TWIN_for_ESP32_20220717a By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Teensy4.0 - (SPI) - ESP32DevKitC - (Wifi/UDP) - PC/python
// ESP32用のマルチスレッド化したMeiridian_core
// Meridian Board Type.K に搭載するESP32に対応。
// PS4コントローラ再対応 https://qiita.com/Ninagawa_Izumi/items/d8966092fe2abd3cba79
// UDP送受とBT受信をスレッド化し、フラグによりデータの流れを円滑化
// IP固定化の要素を追加
// 20220717a もろもろ追加。全体を通じてTeensyのフレームスキップエラーがなおると見通しが良くなる。
// ↑ESP32がPCから同じデータを何度もUDP受信しているように振る舞うところに問題ありそう。

// Meridan TWIN
// ESP32用スケッチ　20220716版
#define VERSION "Hello, This is Meridian_TWIN_for_ESP32_20220717a." // バージョン表示


//================================================================================================================
//---- E S P 3 2 の 配 線  ----------------------------------------------------------------------------------------
//================================================================================================================
/*
 ESP32devkitC  -  Teensy4.0 // あとで書く
   3V3         -  xxxxxxxx
*/


//---------------------------------------------------
// [SETTING] 各種設定 (ES-1) -------------------------
//---------------------------------------------------

/* 頻繁に変更するであろう変数 #DEFINE */
#define FRAME_DURATION 10               // 1フレームあたりの単位時間（単位ms）
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define BT_MAC_ADDR "xx:xx:xx:xx:xx:xx" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
// IPを固定する場合は下記の4項目を設定し、(ES-4-1) WiFi.configを有効にする 固定しない場合はコメントアウト必須
// IPAddress ip(192,168,xx,xx);//ESP32のIPアドレスを固定する場合のアドレス
// IPAddress subnet(255,255,255,0);//ESP32のIPアドレス固定する場合のサブネット
// IPAddress gateway(192,168,xx,xx);//ルーターのゲートウェイを入れる
// IPAddress DNS(8,8,8,8);//DNSサーバーの設定（使用せず）

// (ES-1-2) シリアルモニタリング切り替え
//-特になし-

// (ES-1-3) マウント有無とピンアサイン
//-特になし-

// (ES-1-4) 各種初期設定
#define JOYPAD_MOUNT 4    // ジョイパッドの搭載 (現在2のKRC-5FHのみ有効, ジョイパッドを接続しない場合は0)
                          // 0:なし、1:SBDBT(未), 2:KRC-5FH, 3:PS3(未), 4:PS4(未),5:Wii_yoko, 6:Wii+Nun(未), 7:WiiPRO(未), 8:Xbox(未)
#define JOYPAD_POLLING 10 // ジョイパッドの問い合わせフレーム間隔(PSは10)
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）

bool UDP_SEND = true;     // PCへのデータ送信を行うか
bool UDP_RESEIVE = true;  // PCからのデータ受信を行うか

// (ES-1-5) その他固定値
/* 各種設定 #DEFINE */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define SERIAL_PC 500000 //ESP-PC間のシリアル速度（モニタリング表示用）

#define SEND_PORT 22222         //送り先のポート番号
#define RESV_PORT 22224         //このESP32のポート番号

#define REMOVE_BONDED_DEVICES 0 //0でバインドデバイス情報表示、1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)
#define PAIR_MAX_DEVICES 20 //接続デバイスの記憶可能数


//---------------------------------------------------
// [LIBRARY] ライブラリ関連 (ES-2-LIB) ----------------
//---------------------------------------------------

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* ライブラリ導入 */

// (ES-2-1) ライブラリ全般
#include <Arduino.h>
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
WiFiUDP udp;//wifi設定
#include <ESP32DMASPISlave.h>
ESP32DMASPI::Slave slave;
#include <PS4Controller.h> //PS4コントローラー
#include <ESP32Wiimote.h>  //Wiiコントローラー
ESP32Wiimote wiimote;
//#include <FastCRC.h>
#include <SPI.h>     // SPI自体は使用しないがBNO055に必要

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

/* 変数一般 */

// (ES-3-1) 変数一般
const int MSG_BUFF = MSG_SIZE * 2; // Meridim配列のバイト長

int checksum; //チェックサム計算用
long frame_count = 0;
long error_count_udp = 0;//UDP受信エラーカウント用
long error_count_spi = 0;//SPI受信エラーカウント用
long error_count_esp_skip = 0;//ESPのPCからの受信連番カウントエラー用
uint8_t* s_spi_meridim_dma;//DMA用
uint8_t* r_spi_meridim_dma;//DMA用
TaskHandle_t thp[4];//マルチスレッドのタスクハンドル格納用
int joypad_frame_count; //ジョイパッドのポーリングカウント


// (ES-3-2) フラグ関連
bool udp_rsvd_flag = true;//UDPスレッドでの受信完了フラグ
bool udp_revd_process_queue_flag = false;//UDP受信済みデータの処理待ちフラグ
bool udp_send_flag = false;//UDP送信完了フラグ
bool spi_revd_flag = false;//SPI受信完了フラグ
bool udp_receiving_flag = false;//UDPスレッドでの受信中フラグ（送信抑制）
short frame_sync_s = -30000;//フレーム毎に-30000~29999をカウントし、送信用Meridm[1]に格納
short frame_sync_r_expect = -30000;//フレーム毎に前回受信値に+１として受信値と比較（-30000~29999)
short frame_sync_r_resv = 0;//今フレームに受信したframe_sync_rを格納
short frame_sync_r_past = 0;//前フレームに受信したframe_sync_rを格納

bool udp_sending_flag = false; // UDPスレッドでの送信中フラグ（受信抑制）

/* Meridim配列用の共用体の設定 */ 
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
UnionData r_udp_temp;    //UDP受信確認用
UnionData pad_bt_meridim; // リモコンのBT受信用共用体のインスタンスを宣言

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



//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// +----------------------------------------------------------------------
// | 関数名　　:  checksum_val(short arr[], int len)
// +----------------------------------------------------------------------
// | 機能     :  配列のチェックサムを算出
// | 　　     :  チェックサムは配列の末尾を除く合計数をビット反転しShort型にしたもの
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  short型. チェックサム値
// +----------------------------------------------------------------------
short checksum_val(short arr[], int len)
{
  int cksm = 0;
  for (int i = 0; i < len - 1; i++)
  {
    cksm += int(arr[i]);
  }
  return short(~cksm);
}

// +-------------------------------------------------------------------
// | 関数名　　:  checksum_rslt(short arr[], int len)
// +-------------------------------------------------------------------
// | 機能     :  チェックサムの合否判定
// | 　　     :  配列の末尾を除く合計数をビット反転しShort型にしたものと、末尾の値を比較
// | 引数１　　:  Meridim配列(Short型の配列)
// | 引数２　　:  配列の長さ
// | 戻り値　　:  bool型. チェックサム値がOKならtrue, NGならfalse
// +------------------------------------------------------------------
bool checksum_rslt(short arr[], int len)
{
  int cksm = 0;
  for (int i = 0; i < len - 1; i++)
  {
    cksm += int(arr[i]);
  }
  if (short(~cksm) == arr[len - 1])
  {
    return true;
  }
  return false;
}

// +----------------------------------------------------------------------
// | 関数名　　:  float2HfShort(float val)
// +----------------------------------------------------------------------
// | 機能　　　:  floatを100倍し小数点以下を四捨五入した整数をshortに収める
// | 引数　　　:  float型. 有効数字　-327.67 ~ 327.67
// | 戻り値　　:  short型. 限界値を超えたら限界値を返す(-32767,32767)
// +----------------------------------------------------------------------
short float2HfShort(float val)
{
  int x = round(val * 100); // floatの小数点以下を四捨五入して整数化
  if (x > 32766)
  {
    x = 32767;
  }
  else if (x < -32766)
  {
    x = -32767;
  }
  return (short)x;
}

// +----------------------------------------------------------------------
// | 関数名　　:  float2HfInt(float val)
// +----------------------------------------------------------------------
// | 機能　　　:  floatを100倍し小数点以下を四捨五入した整数をintに収める
// | 引数　　　:  float型. 有効数字8桁(例:123.45678, -12345.678)
// | 戻り値　　:  int型.
// +----------------------------------------------------------------------
int float2HfInt(float val)
{
  int x = round(val * 100); // 100倍し小数点以下を四捨五入
  return x;
}

// +----------------------------------------------------------------------
// | 関数名　　:  initBluetooth()
// +----------------------------------------------------------------------
// | 機能     :  Bluetoothペアリングを設定する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
bool initBluetooth()
{
  if (!btStart())
  {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK)
  {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK)
  {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

// +----------------------------------------------------------------------
// | 関数名　　:  *bda2str(const uint8_t* bda, char *str, size_t size)
// +----------------------------------------------------------------------
// | 機能     :  Bluetoothペアリングアドレスを取得する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
char *bda2str(const uint8_t *bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18)
  {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}


// +----------------------------------------------------------------------
// | 関数名　　:  PS4pad_receive()
// +----------------------------------------------------------------------
// | 機能     :  PS4リモコンの入力値を受信し、以下に値を格納する
// | 　　        pad_btn, pad_L2_val, pad_R2_val, pad_stick_L , pad_stick_R, pad_stick_V
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void PS4pad_receive()
{
  // Below has all accessible outputs from the controller
  if (PS4.isConnected())
  {
    pad_btn = 0;

    if (PS4.Right())
      pad_btn |= (B00000000 * 256) + B00100000;
    if (PS4.Down())
      pad_btn |= (B00000000 * 256) + B01000000;
    if (PS4.Up())
      pad_btn |= (B00000000 * 256) + B00010000;
    if (PS4.Left())
      pad_btn |= (B00000000 * 256) + B10000000;

    if (PS4.Square())
      pad_btn |= (B10000000 * 256) + B00000000;
    if (PS4.Cross())
      pad_btn |= (B01000000 * 256) + B00000000;
    if (PS4.Circle())
      pad_btn |= (B00100000 * 256) + B00000000;
    if (PS4.Triangle())
      pad_btn |= (B00010000 * 256) + B00000000;

    if (PS4.UpRight())
      pad_btn |= (B00000000 * 256) + B00110000;
    if (PS4.DownRight())
      pad_btn |= (B00000000 * 256) + B01100000;
    if (PS4.UpLeft())
      pad_btn |= (B00000000 * 256) + B10010000;
    if (PS4.DownLeft())
      pad_btn |= (B00000000 * 256) + B11000000;

    if (PS4.L1())
      pad_btn |= (B00000100 * 256) + B00000000;
    if (PS4.R1())
      pad_btn |= (B00001000 * 256) + B00000000;

    if (PS4.Share())
      pad_btn |= (B00000000 * 256) + B00000001;
    if (PS4.Options())
      pad_btn |= (B00000000 * 256) + B00001000;
    if (PS4.L3())
      pad_btn |= (B00000000 * 256) + B00000100;
    if (PS4.R3())
      pad_btn |= (B00000000 * 256) + B00000010;

    if (PS4.PSButton())
      pad_btn |= (B00000000 * 256) + B01010000; // same as up & down
    if (PS4.Touchpad())
      pad_btn |= (B00000000 * 256) + B00101000; // same as left & right

    if (PS4.L2())
    {
      pad_btn |= (0x00000001 * 256) + B00000000;
      pad_L2_val = constrain(PS4.L2Value(), 0, 255);
    }
    if (PS4.R2())
    {
      pad_btn |= 512; //(0x00000010 * 256) + B00000000;
      pad_R2_val = constrain(PS4.R2Value(), 0, 255);
    }

    if (PS4.LStickX())
    {
      pad_stick_L_x = constrain(PS4.LStickX(), -127, 127);
    }
    if (PS4.LStickY())
    {
      pad_stick_L_y = constrain(PS4.LStickY(), -127, 127);
    }
    if (PS4.RStickX())
    {
      pad_stick_R_x = constrain(PS4.RStickX(), -127, 127);
    }
    if (PS4.RStickY())
    {
      pad_stick_R_y = constrain(PS4.RStickY(), -127, 127);
    }

    pad_stick_L = pad_stick_L_x * 256 + pad_stick_L_y;
    pad_stick_R = pad_stick_R_x * 256 + pad_stick_R_y;
    pad_stick_V = pad_L2_val * 256 + pad_R2_val;
  }
}

// +----------------------------------------------------------------------
// | 関数名　　:  Wiipad_receive_h()
// +----------------------------------------------------------------------
// | 機能     :  Wiiコントローラ(横持ち・横持ち+ヌンチャク)の入力値を受信し、以下に値を格納する
// | 　　        pad_btn, pad_stick_L
// | 引数　　　:  なし
// +----------------------------------------------------------------------
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

    if (JOYPAD_MOUNT == 6) {
      NunchukState nunchuk = wiimote.getNunchukState();
      int calib_l1x = 5;//LスティックX軸のセンターのキャリブレーション値
      int calib_l1y = -6;//LスティックY軸のセンターのキャリブレーション値
      pad_stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
      //if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      //if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
    Serial.print(127 - nunchuk.xStick);
    Serial.print(",");
    Serial.println(nunchuk.yStick - 127);

    delay(1);//ここの数値でCPU負荷を軽減できるかも
  }
}




// +----------------------------------------------------------------------
// | 関数名　　:  sendUDP()
// +----------------------------------------------------------------------
// | 機能     :  共用体s_udp_meridimをUDP通信で送信する
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void sendUDP()
{
  udp.beginPacket(SEND_IP, SEND_PORT); // UDPパケットの開始
  udp.write(s_udp_meridim.bval, MSG_BUFF);
  udp.endPacket(); // UDPパケットの終了
}

// +----------------------------------------------------------------------
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  UDP通信の受信パケットを確認する。
// | 　　        受信完了を検出したら共用体 r_udp_meridim に値を格納し、
// | 　　        udp_rsvd_flag のフラグをtrueにする
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void receiveUDP()
{
  if (udp.parsePacket() >= MSG_BUFF)        // データの受信バッファ確認
  {
    udp_receiving_flag = true;              // 受信中のセマフォ的フラグ上げ
    delayMicroseconds(5);
    udp.read(r_udp_meridim.bval, MSG_BUFF); // データの受信
    delayMicroseconds(5);
    udp_receiving_flag = false;             // 受信中のセマフォ的フラグさげ
    udp_rsvd_flag = true;                   // 受信完了フラグを上げる
  // delay(1);
  }
}

//-------------------------------------------------------------------------
//---- U D P 受 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_r(void *args) // サブCPU(Core0)で実行するプログラム
{
  delay(1000);
  while (1)
  {
    while (udp_sending_flag != true)
    {
      receiveUDP(); // 常にUDPの受信を待ち受けし、受信が完了したらフラグを挙げる
      delay(1);     // 1/1000秒待つ
    }
    delay(1); // 1/1000秒待つ
  }
}

//-------------------------------------------------------------------------
//---- Bluetooth 用 ス レ ッ ド --------------------------------------------
//-------------------------------------------------------------------------
void Core0_BT_r(void *args) {//サブCPU(Core0)で実行するプログラム
  while (true) {//ここで無限ループを作っておく
    //コントローラの受信ループ
    if (JOYPAD_MOUNT == 4) {
      PS4pad_receive();
    }
    //Wiiコントローラの受信ループ
    if ((JOYPAD_MOUNT == 5) or (JOYPAD_MOUNT == 6)) {
      Wiipad_receive_h();
    }
    delay(1);//JOYPAD_POLLING ms秒待つ
  }
}


void setup()
{
  //-------------------------------------------------------------------------
  //---- (ES-4) 起動時設定 --------------------------------------------------
  //-------------------------------------------------------------------------
  /* PC用シリアルの設定 */ // (ES-4-2) シリアル設定
  Serial.begin(SERIAL_PC);
  delay(130); //シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println();
  Serial.println("Serial Start...");
  Serial.println();
  Serial.println(VERSION);
  delay(100);

  /* WiFiの初期化と開始 */
  // (ES-4-1) WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  //WiFi.config(ip, gateway, subnet, DNS);//※IPを固定にする場合はコメントアウトをはずして有効にする(ES-1-1)も変更すること
  WiFi.begin(AP_SSID, AP_PASS);//WiFiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50);//接続が完了するまでループで待つ
  }

  // (ES-4-3) シリアル表示
  Serial.println(VERSION); //バージョン表示
  Serial.println("WiFi connected to  => " + String(AP_SSID)); // WiFi接続完了通知
  Serial.print("PC's IP address is  => ");
  Serial.println(SEND_IP); // 送信先PCのIPアドレスの表示
  Serial.print("ESP32's IP address is  => ");//ESP32自身のIPアドレスの表示
  Serial.println(WiFi.localIP());

  // (ES-4-6) UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

  /* Bluetoothの初期化 */
  initBluetooth();
  Serial.print("ESP32's Bluetooth Mac Address is => "); // ESP32自身のBluetoothMacアドレスを表示
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));

  Serial.println();
  delay(1000);

  // (ES-4-4) BTペアリング情報
  int bt_count = esp_bt_gap_get_bond_device_num();
  if (!bt_count)
  {
    Serial.println("No bonded BT device found.");
  }
  else
  {
    Serial.print("Bonded BT device count: ");
    Serial.println(bt_count);
    if (PAIR_MAX_DEVICES < bt_count)
    {
      bt_count = PAIR_MAX_DEVICES;
      Serial.print("Reset bonded device count: ");
      Serial.println(bt_count);
    }
    esp_err_t tError = esp_bt_gap_get_bond_device_list(&bt_count, pairedDeviceBtAddr);
    if (ESP_OK == tError)
    {
      for (int i = 0; i < bt_count; i++)
      {
        Serial.print("Found bonded BT device # ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
        if (REMOVE_BONDED_DEVICES)
        {
          esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
          if (ESP_OK == tError)
          {
            Serial.print("Removed bonded BT device # ");
          }
          else
          {
            Serial.print("Failed to remove bonded BT device # ");
          }
          Serial.println(i);
        }
      }
    }
  }


  // (ES-4-5-1) PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4)
  {
    PS4.begin(BT_MAC_ADDR); // ESP32のMACが入ります.PS4にも設定します。
    Serial.println("PS4 controller connecting...");
  }
  
  // (ES-4-5-2) Wiiコントローラの接続開始
  if ((JOYPAD_MOUNT == 5) or (JOYPAD_MOUNT == 6))
  {
    wiimote.init();
    // wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
    Serial.println("Wiimote connecting...");
  }
  
  // (ES-4-7) DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  //※バッファサイズは4で割り切れる必要があり、なおかつ末尾に4バイト分0が入る不具合があるのでその対策

  // (ES-4-8) 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4);// ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);

  // (ES-4-9) 初回の送信データを作成してセット
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    s_spi_meridim.sval[i] = 0;//配列に0を入れる
  }
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE); //データ末尾にチェックサムを入れる
  for (int i = 0; i < MSG_BUFF + 4; i++) { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
  }

  /* スレッドの開始 */
  // (ES-4-10) マルチスレッドの宣言（無線系はすべてCORE0で動くとのこと.メインループはCORE1）
  xTaskCreatePinnedToCore(Core0_UDP_r, "Core0_UDP_r", 4096, NULL, 10, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);

  // (ES-4-11) SPI通信の設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1); // キューサイズ　とりあえず1
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  delay(100);

}

//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //----  [ 1 ]  U D P 受 信  --------------------------------------------------

  //[1-2] UDP受信配列からSPI送信配列にデータを転写.
  if (udp_rsvd_flag) {//UDP受信完了フラグをチェック.(UDP受信データ r_udp_meridim の更新が完了している状態か?)
    udp_rsvd_flag = false;//UDP受信完了フラグを下げる.


    // [1-2-1] UDP受信データ r_udp_meridim のチェックサムを確認.
    if (checksum_rslt(r_udp_meridim.sval, MSG_SIZE))
    {
      // [1-2-1-a-1] 受信成功ならUDP受信データをSPI送信データに上書き更新する.
      memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
      delayMicroseconds(2);

      // このパートのエラーフラグを下げる
      s_spi_meridim.bval[177] &= 0B10111111;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを下げる.

    } else {
      // [1-2-1-b] 受信失敗ならUDP受信データをSPI送信データに上書き更新せず, 前回のSPI送信データにエラーフラグだけ上乗せする.
      s_spi_meridim.bval[177] |= 0B01000000;//meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを上げる.
      error_count_udp ++;//エラーカウンタをアップ.

    }
  }

    // @ [8-1] 通信エラー処理(スキップ検出)
  frame_sync_r_expect++; //フレームカウント予想値を加算
  if (frame_sync_r_expect > 29999)
  { //予想値が29,999以上ならカウントを-30000に戻す
    frame_sync_r_expect = -30000;
  }
  // frame_sync_rの確認(受信フレームにスキップが生じていないかをMeridim[1]のカウンターで判断)
  frame_sync_r_resv = r_udp_meridim.sval[1]; //数値を受け取る

  if (frame_sync_r_resv == frame_sync_r_expect) // frame_sync_rの受信値が期待通りなら順番どおり受信
  {
    s_spi_meridim.bval[MSG_SIZE*2 - 3] &= B11111011; //エラーフラグ10番(ESP受信のスキップ検出)をオフ
  }
  else
  {
    s_spi_meridim.bval[MSG_SIZE*2 - 3] |= B00000100; //エラーフラグ10番(ESP受信のスキップ検出)をオン
    frame_sync_r_expect = frame_sync_r_resv; //予想値とズレていたら現在の受信値を予想結果としてキープ
  }

  //  --->> [check!] ここで s_spi_meridim にはチェックサム済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.

  //----  [ 2 ]  S P I 送 信 信 号 作 成  -----------------------------------------------

  //[2-1] ユーザー定義の送信データの書き込み
  //Teensyへ送るデータをこのパートで作成, 書き込み.
  //  --->> [check!] 現在はここでとくに何もしない.

  //[2-2] リモコンデータの書き込み
  s_spi_meridim.sval[15] = r_spi_meridim.sval[15] | pad_btn;
  s_spi_meridim.sval[16] = r_spi_meridim.sval[16] | pad_stick_L;
  s_spi_meridim.sval[17] = r_spi_meridim.sval[17] | pad_stick_R;
  s_spi_meridim.sval[18] = r_spi_meridim.sval[18] | pad_stick_V;
  Serial.println(r_spi_meridim.sval[15]);
  //  --->> [check!] ここでSPI送信データ s_spi_meridim はESP32で受けたリモコンデータが上書きされた状態.

  //[2-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）
  //※すでにPCから受け取った値がs_spi_meridim.sval[1]に入っているのでここでは何もしない。

  //[2-4] チェックサムの追記
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);
  //  --->> [check!] ここでSPI送信データ s_spi_meridim はチェックサムが入り完成している状態.


  //----  [ 3 ]  S P I 送 受 信  -----------------------------------------------
  while (slave.available())//完了した（受信した）トランザクション結果のサイズ
  {

    // ↓↓↓↓ SPI送受信用のデータはここで処理しなくてはならない ↓↓↓↓
    //[3-1] 完成したSPI送信データをDMAに転記
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
    delayMicroseconds(2);
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
    while (udp_receiving_flag) //UDPが受信処理中なら送信しない
    {
      delayMicroseconds(2);
      //delay(1);
    }
    delayMicroseconds(2);

      //while (udp_receiving) {
      //  delay(1);            // 1/1000秒待つ
      //}
      udp_sending_flag = true;      // 送信busyフラグを挙げる.
      sendUDP();
      delayMicroseconds(10);
      udp_sending_flag = false;     // 送信busyフラグを下げる.
    delayMicroseconds(100);                 // 1/10000秒待つ
    }

}
