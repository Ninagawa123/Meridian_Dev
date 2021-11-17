//Meridian_base_211117_for_Teensy
//Teensy 4.0

/*
  Teensy4.0 Pinassign

  [GND]               -> GND
  [00] RX1, CRX2      -> ICS_3rd_TX
  [01] TX1, CTX2      -> ICS_3rd_RX
  [02]                -> LED（lights up when the processing time is not within the specified time.）
  [03]                -> (NeoPixel Data)
  [04]                -> (NeoPixel Clock)
  [05]                -> ICS_Right_EN
  [06]                -> ICS_Left_EN
  [07] RX2            -> ICS_Left_TX
  [08] TX2            -> ICS_Left_RX
  [09]                -> SD_CS (ESP32[15])
  [10] CS             -> SPI_CS (ESP32[15])
  [11] MOSI           -> SPI/SD_MOSI (ESP32[13])
  [12] MISO           -> SPI/SD_MISO (ESP32[12])
  [Vin]               -> 5V
  [AGND]              ->
  [3.3v]              -> GY-521(MPU6050) 3.3Vin
  [23] CRX1           -> ICS_3rd_EN
  [22] CTX1           ->
  [21] RX5            ->
  [20] TX5            ->
  [19] I2C-SCL0       -> GY-521(MPU6050) SCL
  [18] I2C-SDA0       -> GY-521(MPU6050) SDA
  [17] TX4, I2C-SDA1  -> (PC/Raspi etc.)
  [16] RX4, I2C-SCL1  -> (PC/Raspi etc.)
  [15] RX3            -> ICS_Right_TX
  [14] TX3            -> ICS_Right_RX
  [13] SCK(CRX1)      -> SPI/SD_SCK (ESP32[14])

  Servo ID Assign

  ＜ICS_Left 左手 SIO1,SIO2＞
  [01] 左肩ピッチ
  [02] 左肩ロール
  [03] 左肘ヨー
  [04] 左肘ピッチ
  [05] ー
  ＜ICS_Left 左足 SIO3,SIO4＞
  [00] 腰ヨー
  [06] 左股ロール
  [07] 左股ピッチ
  [08] 左膝ピッチ
  [09] 左足首ピッチ
  [10] 左足首ロール

  ＜ICS_Right 右手 SIO5,SIO6＞
  [00] 頭ヨー
  [01] 右肩ピッチ
  [02] 右肩ロール
  [03] 右肘ヨー
  [04] 右肘ピッチ
  [05] ー
  ＜ICS_Right 右足 SIO7,SIO8＞
  [06] 右股ロール
  [07] 右股ピッチ
  [08] 右膝ピッチ
  [09] 右足首ピッチ
  [10] 右足首ロール
*/

#include <Wire.h>//MPU-6050のライブラリ導入
#include <SPI.h>//SDカード用のライブラリ導入
#include <SD.h>//SDカード用のライブラリ導入
#include <TsyDMASPI.h>//SPI Master用のライブラリを導入
#include <MadgwickAHRS.h>//MPU-6050のライブラリ導入
#include <IcsHardSerialClass.h> //ICSサーボのライブラリ導入

//変数の設定
#define MSG_SIZE 92//
static const int MSG_BUFF = MSG_SIZE * 2;
int checksum;
long err_count = 0;
static bool IMUset = 0;// IMU (6050 or 9250)を接続しているかどうか 1=on,0=off

//タイマー用の変数を用意
long frame_ms = 5;// 1フレームあたりの単位時間(μs)
long merc = (long)millis(); // フレーム管理時計の時刻 Meridian Clock.
long curr = (long)millis(); // 現在時刻を取得
long curr_micro = (long)micros(); // 現在時刻を取得

//LEDテスト用
int err_led = 2;

//共用体を宣言 : Meridian配列データ、SPI送受信バッファ配列
typedef union //共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
  short sval[MSG_SIZE];// short型で100個の配列データを持つ
  uint8_t bval[MSG_BUFF]; //1バイト単位で200個の配列データを持つ
} UnionData;

UnionData s_merdim; //Meridian配列データ(short型、センサや角度は100倍値)
UnionData r_merdim; //Meridian配列データ(short型、センサや角度は100倍値)
UnionData s_packet; //SPI送信用の共用体のインスタンスを作成
UnionData r_packet; //SPI受信用の共用体のインスタンスを作成

//作業用変数
int temp = 0;

//通信のエラーカウント
long error = 0;
long trial = 0;

//SDカード用の変数
const int chipSelect_SD = 9;
bool file_open = 0;

//MPU-6050のアドレス、レジスタ設定値
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
float ROLL, PITCH, YAW, YAW_ZERO;

//ICSサーボ設定用変数
const byte EN_L_PIN = 6;
const byte EN_R_PIN = 5;
const long BAUDRATE = 1250000;
const int TIMEOUT = 1000; //返信待ちのタイムアウト時間。通信できてないか確認する場合には1000ぐらいに設定するとよい。
IcsHardSerialClass krs_L(&Serial2, EN_L_PIN, BAUDRATE, TIMEOUT); //インスタンス＋ENピン(23番ピン)およびUARTの指定
IcsHardSerialClass krs_R(&Serial3, EN_R_PIN, BAUDRATE, TIMEOUT); //インスタンス＋ENピン(2番ピン)およびUARTの指定

//KRSサーボのポジション用配列(degreeではなくサーボ値が入る)
int s_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //14要素
int s_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //14要素
int r_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //14要素
int r_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //14要素

//各サーボの各サーボのマウント判定用配列
bool idl_mt[15];//idlのありなし（mtはmountの略）
bool idr_mt[15];//idrのありなし（mtはmountの略）

//各サーボの内外回転プラマイ方向補正用配列
float idl_pn[15];//idlのプラマイ（pnはposi/negaの略）
float idr_pn[15];//idrのプラマイ（pnはposi/negaの略）

//各サーボの直立デフォルト値(KRS)※20210815変更
int idl_n[15];//idlのニュートラル補正値（nはneutralの略）Meridimで0を指定したときの位置。きおつけ直立とする。
int idr_n[15];//idrのニュートラル補正値（nはneutralの略）

//各サーボのポジション値（中央値を0とした時の増減合計値）
float idl_d[15];//idlの増減分（d）
float idr_d[15];//idrの増減分（d）

//各サーボの計算用変数
int k;

//増分など
int framecount = 0;//サイン計算用の変数
int framecount_diff = 2;//サインカーブ動作などのフレームカウントをいくつずつ進めるか

//************************************************
//*********** 各 種 モ ー ド 設 定  ****************
//************** 0:OFF, 1:ON  ********************
//************************************************
bool trim_adjust = 0; //トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
bool krc_5fh = 0; //KRC-5FHリモコンのデータを接続しているときは1
bool monitor_src = 0; //Teensyでのシリアル表示:送信ソースデータ
bool monitor_send = 0; //Teensyでのシリアル表示:送信データ
bool monitor_resv = 0; //Teensyでのシリアル表示:受信データ
bool monitor_resv_check = 0; //Teensyでのシリアル表示:受信成功の可否
bool monitor_resv_error = 1; //Teensyでのシリアル表示:受信エラー率
bool monitor_all_error = 0; //Teensyでのシリアル表示:全経路の受信エラー率
bool monitor_rpy = 0; //Teensyでのシリアル表示:IMUからのrpy換算値
bool monitor_krc_5fh = 0; //Teensyでのシリアル表示:KRC-5FHリモコンのデータ

void setup() {
  //ユーザー設定
  //各サーボのマウントありなし（1:サーボあり、2:サーボなし）
  idl_mt[0]  = 0; //腰ヨー
  idl_mt[1]  = 1; //左肩ピッチ
  idl_mt[2]  = 1; //左肩ロール
  idl_mt[3]  = 0; //左肘ヨー
  idl_mt[4]  = 1; //左肘ピッチ
  idl_mt[5]  = 0; //左股ヨー
  idl_mt[6]  = 1; //左股ロール
  idl_mt[7]  = 1; //左股ピッチ
  idl_mt[8]  = 1; //左膝ピッチ
  idl_mt[9]  = 1; //左足首ピッチ
  idl_mt[10] = 1; //左足首ロール
  idl_mt[11] = 0; //追加テスト用
  idl_mt[12] = 0; //追加テスト用
  idl_mt[13] = 0; //追加テスト用
  idl_mt[14] = 0; //追加テスト用
  idr_mt[0]  = 1; //頭ヨー
  idr_mt[1]  = 1; //右肩ピッチ
  idr_mt[2]  = 1; //右肩ロール
  idr_mt[3]  = 0; //右肘ヨー
  idr_mt[4]  = 1; //右肘ピッチ
  idr_mt[5]  = 0; //右股ヨー
  idr_mt[6]  = 1; //右股ロール
  idr_mt[7]  = 1; //右股ピッチ
  idr_mt[8]  = 1; //右膝ピッチ
  idr_mt[9]  = 1; //右足首ピッチ
  idr_mt[10] = 1; //右足首ロール
  idr_mt[11] = 0; //追加テスト用
  idr_mt[12] = 0; //追加テスト用
  idr_mt[13] = 0; //追加テスト用
  idr_mt[14] = 0; //追加テスト用

  //各サーボの内外回転プラマイ方向補正
  idl_pn[0]  = +1;//腰ヨー
  idl_pn[1]  = +1;//左肩ピッチ
  idl_pn[2]  = +1;//左肩ロール
  idl_pn[3]  = +1;//左肘ヨー
  idl_pn[4]  = +1;//左肘ピッチ
  idl_pn[5]  = +1;//左股ヨー
  idl_pn[6]  = +1;//左股ロール
  idl_pn[7]  = +1;//左股ピッチ
  idl_pn[8]  = +1;//左膝ピッチ
  idl_pn[9]  = +1;//左足首ピッチ
  idl_pn[10] = +1;//左足首ロール
  idl_pn[11] = +1;//追加テスト用
  idl_pn[12] = +1;//追加テスト用
  idl_pn[13] = +1;//追加テスト用
  idl_pn[14] = +1;//追加テスト用
  idr_pn[0]  = +1;//頭ヨー
  idr_pn[1]  = +1;//右肩ピッチ
  idr_pn[2]  = +1;//右肩ロール
  idr_pn[3]  = +1;//右肘ヨー
  idr_pn[4]  = +1;//右肘ピッチ
  idr_pn[5]  = +1;//右股ヨー
  idr_pn[6]  = +1;//右股ロール
  idr_pn[7]  = +1;//右股ピッチ
  idr_pn[8]  = +1;//右膝ピッチ
  idr_pn[9]  = +1;//右足首ピッチ
  idr_pn[10] = +1;//右足首ロール
  idr_pn[11] = +1;//追加テスト用
  idr_pn[12] = +1;//追加テスト用
  idr_pn[13] = +1;//追加テスト用
  idr_pn[14] = +1;//追加テスト用

  //各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375)
  //直立状態になるよう、具体的な数値を入れて現物調整する
  idl_n[0]  = 0;//腰ヨー
  idl_n[1]  = -70; //左肩ピッチ
  idl_n[2]  = -2700 ; //左肩ロール
  idl_n[3]  = 0;//左肘ヨー
  idl_n[4]  = 2666; //左肘ピッチ
  idl_n[5]  = 0; //左股ヨー
  idl_n[6]  = 0; //左股ロール
  idl_n[7]  = -40; //左股ピッチ
  idl_n[8]  = -1720; //左膝ピッチ
  idl_n[9]  = -600; //左足首ピッチ
  idl_n[10] = -20; //左足首ロール
  idl_n[11] = 0; //追加テスト用
  idl_n[12] = 0; //追加テスト用
  idl_n[13] = 0; //追加テスト用
  idl_n[14] = 0; //追加テスト用
  idr_n[0]  = 0; //頭ヨー
  idr_n[1]  = 0; //右肩ピッチ
  idr_n[2]  = -2650; //右肩ロール
  idr_n[3]  = 0;//右肘ヨー
  idr_n[4]  = 2666; //右肘ピッチ
  idr_n[5]  = 0;//右股ヨー
  idr_n[6]  = 50;//右股ロール
  idr_n[7]  = -100;//右股ピッチ
  idr_n[8]  = -1700; //右膝ピッチ
  idr_n[9]  = -600; //右足首ピッチ
  idr_n[10] = -70; //右足首ロール
  idr_n[11] = 0;//追加テスト用
  idr_n[12] = 0;//追加テスト用
  idr_n[13] = 0;//追加テスト用
  idr_n[14] = 0;//追加テスト用

  //PCでのシリアルモニタ表示
  Serial.begin(600000000);//シリアルモニター表示

  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）

  //サーボモータ用のシリアル通信初期設定
  krs_L.begin(); //サーボモータの通信初期設定。Serial2
  krs_R.begin(); //サーボモータの通信初期設定。Serial3

  //トリムモードがオンの時はループ
  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）
  if (trim_adjust == 1) {
    trimadjustment();
  }

  //ESP32との通信用にSPI_MASTERを開始
  TsyDMASPI0.begin(SS, SPISettings(6000000, MSBFIRST, SPI_MODE3)); //moved from "void loop(){}"

  //SDカードの初期化

  //配列のリセット
  memset(s_merdim.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(r_merdim.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(s_packet.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(r_packet.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(idl_d, 0, 15); //配列要素を0でリセット
  memset(idr_d, 0, 15); //配列要素を0でリセット

  //I2CのSETUP
  setupMPU();

  //変数の設定
  YAW_ZERO = 0;
  s_merdim.sval[0] = MSG_SIZE ;//(トップコマンド）

  //起動時のディレイ用mercちょい足し(サーボ起動待ち用)
  merc = merc + 3000;
}

void trimadjustment() {
  while (true) {
    for (int i = 0; i < 15; i++) {
      if (idl_mt[i] == 1) {
        krs_L.setPos(i, 7500 + (idl_n[i]*idl_pn[i]));
      }
      if (idr_mt[i] == 1) {
        krs_R.setPos(i, 7500 + (idr_n[i]*idr_pn[i]));
      }
      delayMicroseconds(2);
    }
    delay(100);
    Serial.println("Trim adjst mode.");
  }
}

int Deg2Krs(float degree, int id_n) { //degreeにはidl_d[i] * idl_pn[i]、id_nにはidl_n[i]を入れる(左の場合は左半身系)
  float x = 7500 + id_n + (degree / 0.03375); //floatの小数点以下を四捨五入して整数化
  //ちなみにこの計算だと0.02度ぐらいからサーボ値には反映される(=0.59で1に繰り上がる)
  if (x > 11500) {
    x = 11500;
  } else if (x < 3500) {
    x = 3500;
  }
  return x ;
}

// ■ KRSをDegreeの数値に変換する関数 ■
float Krs2Deg(int krs, float n, float pn) { //KRS値のほか idl_n[i], idl_pn[i] を入れる(右の場合はidr系)
  float x = (krs - 7500 - n) * 3.375 * pn;//新
  x = x / 100; //小数点以下2桁で取得する　
  return x;
}

// ■ floatを100倍してshortに収める関数 ■  限界を超えたら限界値張り付き。short型で返す
short float2HFshort(float val) {// float to Hundredfold short
  double x = val * 100; //floatの小数点以下を四捨五入して整数化
  if (x > 32767) {
    x = 32767;
  } else if (x < -32768) {
    x = -32768;
  }
  int y = (int)round(x);
  return (short)y;
}

void loop() {
  //Serial.println("test");

  // [1] センサー処理

  if (merc % 10 == 0) {// 10msごとに読み取り
    getYawPitchRoll();
  }

  // [2] コントローラの読み取りおよび動作の設定

  // [3] Teensy内部で位置制御する場合はここで処理

  // <3-1> トップコマンドの判定によりこの工程の実行orスキップを分岐

  // <3-2> 前回のラストに読み込んだサーボ位置をサーボ書き出し用に書き込む
  for (int i = 0; i < 15; i++) {
    s_KRS_servo_pos_L[i] = Deg2Krs(float(r_merdim.sval[i * 2 + 21]) / 100 * idl_pn[i], idl_n[i]); //
    s_KRS_servo_pos_R[i] = Deg2Krs(r_merdim.sval[i * 2 + 51] / 100 * idr_pn[i], idr_n[i]); //
  }

  // <3-3> センサーデータによる動作へのフィードバック

  //中央からの増減分を角度で計算
  // <3-3> 移動時間の決定

  // <3-4> マスターコマンドの処理
  setyaw();

  // [4] サーボコマンドの書き込み

  // <4-1> Meridian92の配列をサーボ命令に変更

  // <4-2> サーボコマンドの配列に書き込み

  // <4-3> サーボデータのICS送信および返り値を取得

  for (int i = 0; i < 11; i ++) {//接続したサーボの数だけ繰り返す。手持ちの最大は15
    idl_d[i] = 0;
    if (idl_mt[i] == true) {
      if (r_merdim.sval[(i * 2) + 20] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_L.setPos(i, s_KRS_servo_pos_L[i]);
      } else
      {
        k = krs_L.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
      }
      idl_d[i] = Krs2Deg(k, idl_n[i], idl_pn[i]);
    }
    delayMicroseconds(2);
  }

  for (int i = 0; i < 11; i ++) {//接続したサーボの数だけ繰り返す
    idr_d[i] = 0;
    if (idr_mt[i] == true) {
      if (r_merdim.sval[(i * 2) + 50] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_R.setPos(i, s_KRS_servo_pos_R[i]);
      } else
      {
        k = krs_R.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得
      }
      idr_d[i] = Krs2Deg(k, idr_n[i], idr_pn[i]);
    }
    delayMicroseconds(2);
  }

  //Serial.println("test4");

  // [5] Meridian92の送信配列を作成する

  // <5-1> トップコマンドを配列格納
  //s_merdim.sval[0] = MSG_SIZE ;//(トップコマンド）

  // <5-2> 移動時間を配列格納
  //s_merdim.sval[1] = 10 ;//(移動時間）

  // <5-3> センサー値を配列格納
  //s_merdim.sval[2] = (short)acc_x * 100 ; //IMU_gyro_x
  //s_merdim.sval[3] = (short)acc_y * 100 ; //IMU_gyro_y
  //s_merdim.sval[4] = (short)acc_z * 100 ; //IMU_gyro_z
  //s_merdim.sval[5] = (short)gyro_x * 100 ; //IMU_acc_x
  //s_merdim.sval[6] = (short)gyro_y * 100 ; //IMU_acc_y
  //s_merdim.sval[7] = (short)gyro_z * 100 ; //IMU_acc_z
  s_merdim.sval[8] = 0 ;//IMU
  s_merdim.sval[9] = 0 ;//IMU
  s_merdim.sval[10] = 0 ;//IMU
  //s_merdim.sval[11] = (short)temp ;//IMU
  s_merdim.sval[12] = float2HFshort(ROLL) ;//Madgwick_roll
  s_merdim.sval[13] = float2HFshort(PITCH)  ;//Madgwick_pitch
  s_merdim.sval[14] = float2HFshort(YAW) ;//Madgwick_yaw

  // <5-4> サーボIDごとにの現在位置もしくは計算を配列にいれる
  for (int i = 0; i < 15; i++) {
    s_merdim.sval[i * 2 + 20] = 0; //各サーボのコマンドをポジション指示(1)にする
    s_merdim.sval[i * 2 + 21] = float2HFshort(idl_d[i]); //s_merdimに最新のサーボ角度degreeを格納

  }
  for (int i = 0; i < 15; i++) {
    s_merdim.sval[i * 2 + 50] = 0; //各サーボのコマンドをポジション指示(1)にする
    s_merdim.sval[i * 2 + 51] = float2HFshort(idr_d[i]); //各サーボの角度をdegで入れる
  }
  // <5-5> カスタムデータを配列格納

  // <5-6> チェックサムを計算
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += s_merdim.sval[i];
  }
  checksum = ~checksum & 0xFFFF;
  s_merdim.sval[MSG_SIZE - 1] = checksum;


  // <5-7> 送信データのSPIバッファへのバイト型書き込み
  for (int i = 0; i < MSG_BUFF; i++) {
    s_packet.bval[i] = s_merdim.bval[i];
  }

  // [6] ESP32とのISPによる送受信処理

  // <6-1> シリアルモニタ表示（送信データ）

  if (monitor_src == 1) {
    Serial.print("   [Src] ");
    for (int i = 0; i < MSG_SIZE; i++) {
      Serial.print(int (s_merdim.sval[i]));
      Serial.print(",");
    }
    Serial.println();
  }

  if (monitor_send == 1) {
    Serial.print("  [Send] ");
    for (int i = 0; i < MSG_BUFF; i++) {
      Serial.print(int (s_packet.bval[i]));
      Serial.print(",");
    }
    Serial.println();
  }


  // <6-2> ESP32へのSPI送信の実行
  TsyDMASPI0.transfer(s_packet.bval, r_packet.bval, MSG_BUFF);


  // <6-3> SPI受信データチェックサム
  long checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) {
    checksum += r_packet.sval[i];
  }
  checksum = ~checksum & 0xFFFF;
  trial ++;
  if (r_packet.sval[MSG_SIZE - 1] == (short)checksum)//チェックがOKならバッファから受信配列に転記
  {
    if (monitor_resv_check == 1) {
      Serial.println("Rvok! ");//受信OKのシリアル表示
    }
    for (int i = 0; i < MSG_SIZE; i++) {
      r_merdim.sval[i] = r_packet.sval[i];
    }
  } else
  {
    error ++;
    if (monitor_resv_check == 1) {
      Serial.print("RvNG****");//受信のシリアル表示
    }
    if (monitor_resv_error == 1) {//エラー率の表示
      Serial.print("error rate ");
      Serial.print(float(error) / float(trial) * 100);
      Serial.print(" %  ");
      Serial.print(error);
      Serial.print("/");
      Serial.println(trial);
    }
  }

  // <6-4> シリアルモニタ表示（受信データ）
  if (monitor_resv == 1) {
    Serial.print("  [Resv] ");
    for (int i = 0; i < MSG_SIZE; i++) {
      Serial.print(int (r_merdim.sval[i]));
      Serial.print(",");
    }
    Serial.println();
  }

  // [7] フレーム終端処理
  // <7-1> この時点で１フレーム内に処理が収まっていない時の処理
  curr = (long)millis(); // 現在時刻を更新
  if (curr > merc) { // 現在時刻がフレーム管理時計を超えていたらアラートを出す
    //シリアルに遅延msを表示
    Serial.println("*** processing delay :");
    Serial.println(curr - merc);
    digitalWrite(err_led, HIGH);//処理落ちが発生していたらLEDを点灯
  }
  else {
    digitalWrite(err_led, LOW);//処理が収まっていればLEDを消灯
  }

  // <7-2> この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
  curr = (long)millis();
  curr_micro = (long)micros(); // 現在時刻を取得
  //Serial.println(merc * 1000 - curr_micro); //詳細な残り時間をμ秒単位でシリアル表示
  while (curr < merc) {
    curr = (long)millis();
  }

  // <7-3> フレーム管理時計mercのカウントアップ
  merc = merc + frame_ms;//フレーム管理時計を1フレーム分進める
  framecount = framecount + framecount_diff;//サインカーブ動作用のフレームカウントをいくつずつ進めるかをここで設定。
}

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1745);
  mpu.setYAccelOffset(-1034);
  mpu.setZAccelOffset(966);
  mpu.setXGyroOffset(176);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

void getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ROLL = ypr[2] * 180 / M_PI;
    PITCH = ypr[1] * 180 / M_PI;
    YAW = (ypr[0] * 180 / M_PI) - YAW_ZERO;

    if (monitor_rpy == 1) { //Teensyでのシリアル表示:IMUからのrpy換算値
      Serial.print("[Roll, Pitch, Yaw] ");
      Serial.print(ROLL);
      Serial.print(", ");
      Serial.print(PITCH);
      Serial.print(", ");
      Serial.println(YAW);
    }
  }
}

void setyaw() {
  if (r_merdim.sval[0] == 2) { //
    YAW_ZERO = ypr[0] * 180 / M_PI;
    s_merdim.sval[0] == MSG_SIZE;
  }
}
