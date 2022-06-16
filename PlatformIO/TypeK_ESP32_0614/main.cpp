
// Meridian_TWIN_for_ESP32_20220615 By Izumi Ninagawa & Meridian Project
// MIT Licenced.
//
// Teensy4.0 - (SPI) - ESP32DevKitC - (Wifi/UDP) - PC/python
// ESP32用のマルチスレッド化したMeiridian_core
// Meridian Board Type.K に搭載するESP32に対応。
// PS4コントローラ再対応 https://qiita.com/Ninagawa_Izumi/items/d8966092fe2abd3cba79
// UDP送受、UDP送信、両方ともスレッド化し、フラグによりデータの流れを円滑化
// ただしBTを同時接続するとエラーでデータが欠損するので、リモコンは一旦わすれる。
// リモコンの運用は今後、PC-UDP接続時はPC側でリモコン処理、
// PCレスのスタンドアロン時はESP32でBTを使い、Wiiをキャンセル（未導入）で対応する。
// IP固定化の要素を追加。

// Meridan Board -LITE-
// ESP32用スケッチ　20220614版
#define VERSION "Hello, This is Meridian_TWIN_for_ESP32_20220615." // バージョン表示

// 100Hz通信で安定動作。
// 現在使える構成要素は
// ① ICSサーボL,R 11個ずつ合計22軸分。
// ② リモコン KRC-5FH + KRR-5FH
// ③ 9軸センサ BNO005
// ④ SDカード(SPIに接続)

// ESP32 DevkitC + PlatformIOで使うことを想定
// PlatformIO上でのESP32のボードバージョンは3.5.0が良い。（4系はうまく動かないらしい）
// Serial1を使うには設定を変更する必要あり. 参考 → https://qiita.com/Ninagawa_Izumi/items/8ce2d55728fd5973087d

// サーボ値の返信は取りこぼしなどあるが、取りこぼす直線のデータを使うことで擬似的に安定させている。
// そのため返信値は参考値であり、基本的にはサーボ値を指示して動作させることが基本的な利用法となる。

//================================================================================================================
//---- E S P 3 2 の 配 線  ----------------------------------------------------------------------------------------
//================================================================================================================
/*
 ESP32devkitC  -  デバイス
   3V3         -  BNO005 VIN
   21          -  BNO005 SCL
   22          -  BNO005 SDA
   GND         -  BNO005 GND

   4  EN       -  ICS変換基板 R系統 EN
   16 RX       -  ICS変換基板 R系統 TX
   17 TX       -  ICS変換基板 R系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND

   33 EN       -  ICS変換基板 L系統 EN
   32 RX       -  ICS変換基板 L系統 TX
   27 TX       -  ICS変換基板 L系統 RX
   5V          -  ICS変換基板 IOREF
   GND         -  ICS変換基板 GND
*/

//================================================================================================================
//---- 初 期 設 定  -----------------------------------------------------------------------------------------------
//================================================================================================================

/* ライブラリ導入 */
#include <Arduino.h>
#include <WiFi.h>    // WiFi通信用ライブラリ
#include <WiFiUdp.h> // UDP通信用ライブラリ
#include <Wire.h>    // I2C通信用ライブラリ
#include <SPI.h>     // SPI自体は使用しないがBNO055に必要
#include <SD.h>      //SDカード用のライブラリ導入

#include <ESP32DMASPISlave.h>
ESP32DMASPI::Slave slave;
#include <PS4Controller.h> //PS4コントローラー
#include <ESP32Wiimote.h>  //Wiiコントローラー
ESP32Wiimote wiimote;

// (ES-2-2) PS4用を新規接続するためのペアリング情報解除設定
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

/* 頻繁に変更するであろう変数 #DEFINE */
#define FRAME_DURATION 10               // 1フレームあたりの単位時間（単位ms）
#define AP_SSID "bishamon2"             // アクセスポイントのAP_SSID
#define AP_PASS "c6edKxHU"              // アクセスポイントのパスワード
#define SEND_IP "192.168.1.25"          // 送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define BT_MAC_ADDR "ec:94:cb:6f:c6:c6" // ESP32自身のBluetoothMACアドレス（本プログラムを実行しシリアルモニタで確認）
// IPを固定する場合は下記の4項目を設定し、(ES-4-1) WiFi.configを有効にする 固定しない場合はコメントアウト必須
// IPAddress ip(192,168,xx,xx);//ESP32のIPアドレスを固定する場合のアドレス
// IPAddress subnet(255,255,255,0);//ESP32のIPアドレス固定する場合のサブネット
// IPAddress gateway(192,168,xx,xx);//ルーターのゲートウェイを入れる
// IPAddress DNS(8,8,8,8);//DNSサーバーの設定（使用せず）

#define SD_MOUNT 1        // SDリーダーの搭載 (0:なし, 1:あり)
#define IMU_MOUNT 1       // 6軸or9軸センサーの搭載 (0:なし, 1:BNO055, 2:MPU6050(未実装))
#define JOYPAD_MOUNT 4    // ジョイパッドの搭載 (現在2のKRC-5FHのみ有効, ジョイパッドを接続しない場合は0)
                          // 0:なし、1:SBDBT(未), 2:KRC-5FH, 3:PS3(未), 4:PS4(未),5:Wii_yoko, 6:Wii+Nun(未), 7:WiiPRO(未), 8:Xbox(未)
#define JOYPAD_POLLING 10 // ジョイパッドの問い合わせフレーム間隔(PSは10)
bool UDP_SEND = true;     // PCへのデータ送信を行うか
bool UDP_RESEIVE = true;  // PCからのデータ受信を行うか

/* マスターコマンド定義 */
#define SET_YAW_CENTER 1002  // センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 1003 // トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

/* 各種設定 #DEFINE */
#define MSG_SIZE 90       // Meridim配列の長さ設定（デフォルトは90）
#define SEND_PORT 22222   // 送り先のポート番号
#define RESV_PORT 22224   // このESP32のポート番号
#define ERR_LED 25        // LED用 処理が時間内に収まっていない場合に点灯
#define BAUDRATE1 1250000 // サーボの通信速度
#define TIMEOUT1 2        // サーボ返信エラーをスルーするのに程よい設定が2
#define EN_PIN_L 33       // サーボL系統のENピン
#define EN_PIN_R 4        // サーボR系統のENピン
#define SERVO_NUM_L 11    // L系統につないだサーボの数
#define SERVO_NUM_R 11    // R系統につないだサーボの数
#define CHIPSELECT_SD 15  // SDカード用のCSピン

// (ES-1-4) 各種初期設定
//#define JOYPAD_POLLING 10 ////ジョイパッドの問い合わせフレーム間隔(ms)
#define MSG_SIZE 90                // Meridim配列の長さ設定（デフォルトは90）
#define SERIAL_PC 500000           // ESP-PC間のシリアル速度（モニタリング表示用）
const int MSG_BUFF = MSG_SIZE * 2; // Meridim配列のバイト長

// (ES-1-5) その他固定値
#define SEND_PORT 22222         //送り先のポート番号
#define RESV_PORT 22224         //このESP32のポート番号
#define REMOVE_BONDED_DEVICES 0 // 0でバインドデバイス情報表示、1でバインドデバイス情報クリア(BTリモコンがペアリング接続できない時に使用)
#define PAIR_MAX_DEVICES 20     //接続デバイスの記憶可能数
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];
// Meridim配列の長さ設定（デフォルトは90）

/* マルチスレッド用変数 */
TaskHandle_t thp[4]; //マルチスレッドのタスクハンドル格納用

/* リモコン用変数 */
unsigned short pad_btn = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;//ディスプレイ表示用（バイナリ表示の上位ビット）


// (ES-3-4) コントローラー用変数
//unsigned short pad_btn = 0;
short pad_stick_R = 0;
//short pad_stick_R_x = 0;
//short pad_stick_R_y = 0;
short pad_stick_L = 0;
//short pad_stick_L_x = 0;
//short pad_stick_L_y = 0;
short pad_stick_V = 0;
//short pad_R2_val = 0;
//short pad_L2_val = 0;
//long pad_btn_disp = 65536; 

/* センサー用変数 */
float bno055_read[16]; // bno055のデータの一時保存
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float yaw_center = 0;

/* Meridim配列用の共用体の設定 */ //(ES-3-3) 共用体の設定. 共用体はたとえばデータをショートで格納し,バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_udp_meridim;  // UDP送信用共用体のインスタンスを宣言
UnionData r_udp_meridim;  // UDP受信用共用体のインスタンスを宣言
UnionData s_spi_meridim;  // SPI受信用共用体
UnionData r_spi_meridim;  // SPI受信用共用体
UnionData pad_bt_meridim; // リモコンのBT受信用共用体のインスタンスを宣言

/* DMA用の変数 */
uint8_t *s_spi_meridim_dma; // DMA用
uint8_t *r_spi_meridim_dma; // DMA用

/* 変数一般 */
long frame_count = 0;
long frame_ms = FRAME_DURATION;    // 1フレームあたりの単位時間(ms)
int frame_count_diff = 5;          // サインカーブ等で使う1フレームあたりの増分値
long merc = (long)millis();        // フレーム管理時計の時刻 Meridian Clock.
long curr_millis = (long)millis(); // 現在時刻を取得
long curr_micro = (long)micros();  // 現在時刻を取得
char frame_sync_s = 0;             //フレーム毎に0-199をカウントし、送信用Meridm[88]の下位8ビットに格納
char frame_sync_r_expect = 0;      //フレーム毎に0-199をカウントし、受信値と比較
char frame_sync_r_resv = 0;        //今フレームに受信したframe_sync_rを格納
long error_count_udp = 0;          // UDP受信エラーカウント用
long error_count_spi = 0;          // SPI受信エラーカウント用
long error_count_esp_skip = 0;     // ESPのPCからの受信連番カウントエラー用

// float n;                    //計算用
int joypad_frame_count; //ジョイパッドのポーリングカウント
// bool monitor_joypad = true; //ジョイパッドのモニタリング用（後で消す）

/* フラグ用変数 */          // (ES-3-2) フラグ関連
bool udp_revd_flag = false; // UDPスレッドでの受信完了フラグ
// bool udp_resv_process_queue_flag = false;  // UDP受信済みデータの処理待ちフラグ
// bool udp_send_flag = false;         // UDP送信完了フラグ
// bool spi_revd_flag = false;         // SPI受信完了フラグ
// bool udp_receiving_flag = false;    // UDPスレッドでの受信中フラグ（送信抑制）
bool udp_sending_flag = false; // UDPスレッドでの送信中フラグ（受信抑制）

//

// (ES-3-1) 変数一般
int checksum; //チェックサム計算用
//

/* wifi設定 */
WiFiUDP udp;

//================================================================================================================
//---- 関 数 各 種  -----------------------------------------------------------------------------------------------
//================================================================================================================

// +-------------------------------------------------------------------
// | 関数名　　:  setyawcenter()
// +-------------------------------------------------------------------
// | 機能     :  センサーのヨー軸中央値を再セットする
// +------------------------------------------------------------------
void setyawcenter()
{
  yaw_center = bno055_read[14];
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
// | 関数名　　:  Krs2HfDeg(int krs, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  KRS単位値をdegree値*100に変換. HfはHundredfoldの略.
// | 引数１　　:  int型. サーボ値（KRS単位値）
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. degree値*100
// +----------------------------------------------------------------------
int Krs2HfDeg(int krs, int n, int pn)
{
  float x = (krs - 7500 - n) * 3.375 * pn;
  return int(x);
}

// +----------------------------------------------------------------------
// | 関数名　　:  HfDeg2Krs(int hfdegree, int n, int pn)
// +----------------------------------------------------------------------
// | 機能     :  degree値*100 をKRS単位値に変換. HfはHundredfoldの略.
// | 引数１　　:  float型. degree値
// | 引数２　　:  int型. トリム値（KRS単位値）       具体例：idl_trim[i], idr_trim[i]
// | 引数３　　:  int型. 回転方向の順逆補正（+1,-1）　具体例：idl_cw[i],idr_cw[i]
// | 戻り値　　:  int型. KRS単位値（3500-11500）
// | 備考　　　:  0.02度ぐらいからサーボ値には反映される(=0.59で1に繰り上がる)
// +----------------------------------------------------------------------
int HfDeg2Krs(int hfdegree, int n, int pn)
{
  float x = 7500 + n + (hfdegree / 3.375 * pn); //
  if (x > 11500)                                //上限を設定
  {
    x = 11500;
  }
  else if (x < 3500) //下限を設定
  {
    x = 3500;
  }
  return int(x);
}

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
// | 関数名　　:  receiveUDP()
// +----------------------------------------------------------------------
// | 機能     :  UDP通信の受信パケットを確認する。
// | 　　        受信完了を検出したら共用体 r_udp_meridim に値を格納し、
// | 　　        udp_revd_flag のフラグをtrueにする
// | 引数　　　:  なし
// +----------------------------------------------------------------------
void receiveUDP()
{
  if (udp.parsePacket() >= MSG_BUFF) //データの受信バッファ確認
  {
    udp.read(r_udp_meridim.bval, MSG_BUFF); //データの受信
    udp_revd_flag = true;                   //受信完了フラグを上げる
  }
  // delay(1);
}

// void receiveUDP() {
//   //Serial.println("receiveUDP()");
//   while (udp_sending_flag) {
//     //delayMicroseconds(10);
//     delay(1);
//   }
//   int packetSize = udp.parsePacket();//受信済みのパケットサイズを取得
//   byte tmpbuf[MSG_BUFF];
//
//   //データの受信
//   if (packetSize >= MSG_BUFF) {//受信済みのパケットサイズを確認し、配列分受信できていたら読み出し
//
//     udp_receiving_flag = 1;
//     udp.read(tmpbuf, MSG_BUFF);
//     memcpy(r_udp_meridim.bval, tmpbuf, MSG_BUFF);
//     udp_receiving_flag = false;
//     udp_revd_flag = true;//r_udp_meridimに受信完了した旨のフラグを挙げる
//   }
// }

//-------------------------------------------------------------------------
//---- U D P 受 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_r(void *args)
{ //サブCPU(Core0)で実行するプログラム
  delay(1000);
  while (1)
  {
    while (udp_sending_flag == false)
    {
      receiveUDP(); //常にUDPの受信を待ち受けし、受信が完了したらフラグを挙げる
      delay(1);     // 1/1000秒待つ
    }
    delay(1); // 1/1000秒待つ
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



//-------------------------------------------------------------------------
//---- U D P 送 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
// void Core0_UDP_s(void *args) {//サブCPU(Core0)で実行するプログラム
//  delay(1000);
//  while (1) {
//    if (udp_send_flag) {//メインループにより送信フラグが挙がったら送信実行
//      udp_sending_flag = true;//送信busyフラグを挙げる.
//      sendUDP();
//      delayMicroseconds(10);
//      udp_sending_flag = false;//送信busyフラグを下げる.
//      udp_send_flag = false;
//    }
//    delay(1);//1/1000秒待つ
//  }
//}

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
// | 関数名　　:  Wiipad_receive()
// +----------------------------------------------------------------------
// | 機能     :  Wiiリモコン(横持ち・横持ち+ヌンチャク)の入力値を受信し、以下に値を格納する
// | 　　        pad_btn, pad_stick_L
// | 引数　　　:  なし
// +----------------------------------------------------------------------

void Wiipad_receive()
{
  wiimote.task();
  if (wiimote.available() > 0)
  {
    uint16_t button = wiimote.getButtonState();
    NunchukState nunchuk = wiimote.getNunchukState();
    pad_btn = 0;

    if (button & 0x0400)
      pad_btn |= (B00000000 * 256) + B00100000; // right
    if (button & 0x0100)
      pad_btn |= (B00000000 * 256) + B01000000; // down
    if (button & 0x0200)
      pad_btn |= (B00000000 * 256) + B00010000; // up
    if (button & 0x0800)
      pad_btn |= (B00000000 * 256) + B10000000; // left

    if (button & 0x0008)
      pad_btn |= (B10000000 * 256) + B00000000; // A
    if (button & 0x0002)
      pad_btn |= (B01000000 * 256) + B00000000; // 1
    if (button & 0x0001)
      pad_btn |= (B00100000 * 256) + B00000000; // 2
    if (button & 0x0004)
      pad_btn |= (B00010000 * 256) + B00000000; // B

    if (button & 0x0010)
      pad_btn |= (B00000000 * 256) + B00000001; //+
    if (button & 0x1000)
      pad_btn |= (B00000000 * 256) + B00001000; //-

    if (button & 0x0080)
      pad_btn |= (B00000000 * 256) + B01010000; // same as up & down//home

    if (JOYPAD_MOUNT == 2)
    {
      NunchukState nunchuk = wiimote.getNunchukState();
      int calib_l1x = 5;  // LスティックX軸のセンターのキャリブレーション値
      int calib_l1y = -6; // LスティックY軸のセンターのキャリブレーション値
      pad_stick_L = ((nunchuk.xStick + calib_l1x - 127) * 256 + (nunchuk.yStick - 127 + calib_l1y));
      // if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      // if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
    Serial.print(127 - nunchuk.xStick);
    Serial.print(",");
    Serial.println(nunchuk.yStick - 127);

    delay(1); //ここの数値でCPU負荷を軽減できるかも
  }
}

//-------------------------------------------------------------------------
//---- Bluetooth 用 ス レ ッ ド --------------------------------------------
//-------------------------------------------------------------------------
void Core0_BT_r(void *args)
{ //サブCPU(Core0)で実行するプログラム
  while (1)
  { //ここで無限ループを作っておく
    // PS4コントローラの受信ループ
    if (JOYPAD_MOUNT == 4)
    {
      PS4pad_receive();
    }
    // Wiiコントローラの受信ループ
    if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2))
    {
      Wiipad_receive();
    }
    delay(1); // JOYPAD_POLLING ms秒待つ
  }
}


//================================================================================================================
//---- S E T  U P -----------------------------------------------------------------------------------------------
//================================================================================================================
void setup()
{
  /* ピンモードの設定 */
  pinMode(ERR_LED, OUTPUT); // エラー用LED

  /* PC用シリアルの設定 */ // (ES-4-2) シリアル設定
  Serial.begin(SERIAL_PC);
  delay(130); //シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println();
  Serial.println("Serial Start...");
  Serial.println();
  Serial.println(VERSION);
  Wire.begin(22, 21);

  /* スレッドの開始 */

  /* WiFiの初期化と開始 */     //(ES-4-1) WiFi 初期化
  WiFi.disconnect(true, true); // WiFi接続をリセット
  // WiFi.config(ip, gateway, subnet, DNS);//※IPを固定にする場合はコメントアウトをはずして有効にする(ES-1-1)も変更すること

  // Serial.println("Connecting to WiFi to : " + String(AP_SSID)); //接続先を表示
  delay(100);
  WiFi.begin(AP_SSID, AP_PASS); // Wifiに接続
  while (WiFi.status() != WL_CONNECTED)
  {            // https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50); //接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected to  => " + String(AP_SSID)); // WiFi接続完了通知

  Serial.print("PC's IP address is  => ");
  Serial.println(SEND_IP); // 送信先PCのIPアドレスの表示

  Serial.print("ESP32's IP address is  => ");
  Serial.println(WiFi.localIP()); // ESP32自身のIPアドレスの表示
  udp.begin(RESV_PORT);           // UDP通信の開始

  /* Bluetoothの初期化 */
  initBluetooth();
  Serial.print("ESP32's Bluetooth Mac Address is => "); // ESP32自身のBluetoothMacアドレスを表示
  Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));

  /* Bluetoothの初期化 */
  // uint8_t bt_mac[6];
  // String self_mac_address = "";
  // esp_read_mac(bt_mac, ESP_MAC_BT); // ESP32自身のBluetoothMacアドレスを表示
  // self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  // Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
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
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); // DMAバッファ設定
  //※バッファサイズは4で割り切れる必要があり、なおかつ末尾に4バイト分0が入る不具合があるのでその対策

  // (ES-4-8) 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4); // ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);

  // (ES-4-9) 初回の送信データを作成してセット
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    s_spi_meridim.sval[i] = 0;              //配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
  }
  checksum = short(~checksum);                        //チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum); //データ末尾にチェックサムを入れる
  for (int i = 0; i < MSG_BUFF + 4; i++)
  { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i];
  }

  // (ES-4-10) マルチスレッドの宣言（無線系はすべてCORE0で動くとのこと.メインループはCORE1）
  // xTaskCreatePinnedToCore(Core0_UDP_s, "Core0_UDP_s", 4096, NULL, 10, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0_UDP_r, "Core0_UDP_r", 4096, NULL, 20, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 4096, NULL, 5, &thp[2], 0);

  // (ES-4-11) SPI通信の設定
  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て(1か2のみ)
  slave.setQueueSize(1);  // キューサイズ　とりあえず1
  slave.begin();          // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）ピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12

  merc += (long)millis() + 10; // mercをリセット
}

//================================================================================================================
//---- M A I N  L O O P -----------------------------------------------------------------------------------------
//================================================================================================================
void loop()
{

  //////// < 1 > U D P 受 信 ///////////////////////////////////////////////////////

  // @ [1-1a] UDP受信の実行
  //  --->> [check!] UDP受信は別スレッド void Core0_UDP_r() で実行されている.
  // @ [1-1b] UDP受信完了待ちループ
  while (udp_revd_flag != true)
  {
    delayMicroseconds(100);
    Serial.println("Waiting_udp...");
  }
  Serial.println("Rsvd_udp.");
  udp_revd_flag = false; // UDP受信完了フラグを下げる.

  // @ [1-2] UDP受信配列からSPI送信配列にデータを転写.
  // @ [1-2-1] UDP受信データ r_udp_meridim のチェックサムを確認.

  if (checksum_rslt(r_udp_meridim.sval, MSG_SIZE)) 
  // @ [1-2-1-a-1] UDP受信チェックサムOKの場合
  {
    // UDP受信データをSPI送信データに上書き更新する.
    memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
    delayMicroseconds(1);

    // UDP受信エラーフラグを下げる
    s_spi_meridim.bval[177] &= 0B10111111; // meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを下げる.

    // [1-2-1-a-2] 通信エラー処理(スキップ検出)
    // frame_sync_r_resv = s_spi_meridim.bval[176]; //数値を受け取る

    //予測値のカウントアップ
    //frame_sync_r_expect++;
    //if (frame_sync_r_expect > 199)
    //{
    //  frame_sync_r_expect = 0;
    //}

    //カウント受信の比較
    // Serial.print(int(frame_sync_r_expect));
    // Serial.print(":");
    // Serial.println(int(frame_sync_r_resv));

    //if (frame_sync_r_resv == frame_sync_r_expect) //予想通りなら取りこぼしなし
    //{
    //  s_spi_meridim.bval[177] &= B11111011; //エラーフラグ10番(ESPのPCからのUDP取りこぼし検出)をオフ
    //}
    //else
    //{
    //  frame_sync_r_expect = frame_sync_r_resv; //受信値を正解の予測値だったとする
    //  s_spi_meridim.bval[177] |= B00000100;    //エラーフラグ10番(ESPのPCからのUDP取りこぼし検出)をオン
    //  error_count_esp_skip++;
    //}

    // UDPのスキップ回数を表示
    // Serial.print("  count ");
    // Serial.println(int(error_count_esp_skip));

    //受信カウントをそのまま送信カウントとすることで、PCデータがシーケンシャルにTsyに届いてるかのチェックとする
    //frame_sync_s = frame_sync_r_resv;
  }
  //[1-2-1-b] UDP受信チェックサムNGの場合
  else 
  {
    // UDP受信データをSPI送信データに上書き更新せず, 前回のSPI送信データにエラーフラグだけ上乗せする.
    s_spi_meridim.bval[177] |= B01000000; // meridimの[88]番の14ビット目(ESPのUPD受信成否)のフラグを上げる.
    error_count_udp++;                    //エラーカウンタをアップ.
  }

  // @ [8-1] 通信エラー処理(スキップ検出)
  //frame_sync_r_expect++; //フレームカウント予想値を加算
  //if (frame_sync_r_expect > 199)
  //{ //予想値が200以上ならカウントを0に戻す
  //  frame_sync_r_expect = 0;
  //}
  // frame_sync_rの確認(受信フレームにスキップが生じていないかをMeridim[88]の下位8ビットのカウンターで判断)
  //frame_sync_r_resv = s_udp_meridim.bval[176]; //数値を受け取る

  //if (frame_sync_r_resv == frame_sync_r_expect) // frame_sync_rの受信値が期待通りなら順番どおり受信
  //{
  //  s_udp_meridim.bval[177] &= B11111011; //エラーフラグ10番(ESP受信のスキップ検出)をオフ
  //}
  //else
  //{
  //  s_udp_meridim.bval[177] |= B00000100; //エラーフラグ10番(ESP受信のスキップ検出)をオン
  //  if (frame_sync_r_resv == frame_sync_r_expect - 1)
  //  {
  //    //同じ値を２回取得した場合には実際はシーケンスが進んだものとして補正（アルゴリズム要検討）
  //    frame_sync_r_expect = frame_sync_r_resv + 1;
  //  }
  //  else
  //  {
  //    frame_sync_r_expect = frame_sync_r_resv; //取りこぼしについては現在の受信値を正解の予測値としてキープ
  //  }
  //}
  //  --->> [check!] ここで s_spi_meridim にはチェックサム済みの r_udp_meridim が転記され, ESP32UDP受信エラーフラグも入った状態.


  //////// < 2 > S P I 送 信 信 号 作 成 /////////////////////////////////////////////

  // @ [2-1] ユーザー定義の送信データの書き込み
  // Teensyへ送るデータをこのパートで作成, 書き込み.
  //  --->> [check!] 現在はここでとくに何もしない.

  // @ [2-2] コントロールパッド受信値の転記
  if (JOYPAD_MOUNT == 4)
  {
    //for (int i = 80; i < 84; i++)
    //{
    //  s_udp_meridim.sval[i] = pad_bt_meridim.sval[i];
    //}
  s_spi_meridim.sval[80] = pad_btn;
  s_spi_meridim.sval[81] = pad_stick_L;
  s_spi_meridim.sval[82] = pad_stick_R;
  s_spi_meridim.sval[83] = pad_stick_V;
  }

  //  --->> [check!] ここでSPI送信データ s_spi_meridim はESP32で受けたリモコンデータが上書きされた状態.

  //[2-3] フレームスキップ検出用のカウントを転記して格納（PCからのカウントと同じ値をESPに転送）
  //s_spi_meridim.bval[176] = short(frame_sync_s);

  //[2-4] チェックサムの追記
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);
  //  --->> [check!] ここでSPI送信データ s_spi_meridim はチェックサムが入り完成している状態.

  //////// < 3 > S P I 送 受 信 /////////////////////////////////////////////////////

  while (slave.available()) //完了した（受信した）トランザクション結果のサイズ
  {
    //[3-1] 完成したSPI送信データをDMAに転記
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
    delayMicroseconds(1);
    // Serial.println("[3] copied to SPI_dma");
    slave.pop(); // DMAのデータ配列の中で最新の要素を削除
  }

  //[3-2] SPI送受信の実行
  // SPIの送受信:キューが送信済みであればセットされた送信データを送信する.
  if (slave.remained() == 0)
  { //キューに入れられた（完了していない）トランザクションのサイズが0なら実行.
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    frame_count = frame_count + 1; // SPIの受信をもってフレームカウントとする

    //////// < 1 > U D P 送 信 信 号 作 成 ////////////////////////////////////////////
    //[4-1] 受信したSPI送信データをUDP送信データに転記.
    memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4);
    delayMicroseconds(1);
    //  --->> [check!] UDP送信データ"s_udp_meridim"に中身が入った状態.

    //////// < 2 > U D P 送 信 ///////////////////////////////////////////////////////
    // @ [2-1] UDP送信を実行
    udp_sending_flag = true;
    if (UDP_SEND) //設定でUDPの送信を行うかどうか決定
    {
      sendUDP();
    }
    delayMicroseconds(5);
    udp_sending_flag = false;
    /*
      while (udp_receiving_flag) //UDPが受信処理中なら送信しない
      {
      delayMicroseconds(2);
      }
    */
    // udp_send_flag = true; // UDP送信準備完了フラグを挙げる.(スレッドCore0_UDP_sで送信実行)
  }
}
