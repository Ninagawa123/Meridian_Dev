
// Meridian_core_for_Teensy_2022.05.01 By Izumi Ninagawa & Meridian Project
// MIT Licenced.

//This code is for Teensy 4.0
//PC計測現状100Hz~200HzでPC→Teensyのデータ取りこぼし10%程度,50Hzならそこそこ安定。
//
//MeridianControlPanelDPG対応
//全体的にコードを整理
//2022.05.01 IMUのデータ取得タイミングをタイマー割り込みタイプに変更

//---------------------------------------------------
// [SETTING] 各種設定 (TS-1) -------------------------
//---------------------------------------------------

// (TS-1-1) 変更頻度高め
#define VERSION "Meridian_core_MT_for_Teensy_2022.05.01_kenpin"//バージョン表示
#define FRAME_DURATION 10//1フレームあたりの単位時間（単位ms）

// (TS-1-2) シリアルモニタリング切り替え
#define MONITOR_SRC 0 //Teensyでのシリアル表示:送信ソースデータ
#define MONITOR_SEND 0 //Teensyでのシリアル表示:送信データ
#define MONITOR_RESV 0 //Teensyでのシリアル表示:受信データ
#define MONITOR_RESV_SHORT 0 //Teensyでのシリアル表示:受信データ(short)
#define MONITOR_RESV_CHECK 0 //Teensyでのシリアル表示:受信成功の可否
#define MONITOR_RESV_ERROR 0 //Teensyでのシリアル表示:受信エラー率
#define MONITOR_ALL_ERROR 0 //Teensyでのシリアル表示:全経路の受信エラー率
#define MONITOR_RPY 0 //Teensyでのシリアル表示:IMUからのrpy換算値
#define MONITOR_JOYPAD 0 //Teensyでのシリアル表示:リモコンのデータ

// (TS-1-3) マウント有無とピンアサイン
#define ESP32_MOUNT 1 //ESPの搭載 0:なし(SPI通信およびUDP通信を実施しない), 1:あり
#define IMU_MOUNT 1 //IMUの搭載状況 0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) (※1のみ実装済,MeridianBoardではICS_R系に接続)
#define IMU_FREQ 10 //IMUのセンサの読み取り間隔(ms)
#define JOYPAD_MOUNT 0 //ジョイパッドの搭載 0:なしorESP32orPCで受信, 1:SBDBT, 2:KRC-5FH (※2のみ実装済,MeridianBoardではICS_R系に接続)
#define JOYPAD_FRAME 4 //上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define ICS3_MOUNT 0 //半二重サーボ信号の3系のありなし
#define SD_MOUNT 1 //SDカードリーダーのありなし. MeridianBoard Type.Kは有り
#define CHIPSELECT_SD 9 //SDカードSPI通信用のChipSelectのピン番号

// (TS-1-4) マスターコマンド定義
#define TRIM_ADJUST_MODE 0 //トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
#define UPDATE_YAW_CENTER 1002 //センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 1003   //トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

// (TS-1-5) その他固定値
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define ERR_LED 2 //LED用 処理が時間内に収まっていない場合に点灯
#define EN_L_PIN 6 //ICSサーボ信号の左系のENピン番号（固定）
#define EN_R_PIN 5 //ICSサーボ信号の右系のENピン番号（固定）
#define EN_3_PIN 23 //半二重サーボ信号の3系のENピン番号（固定）
#define SERIAL_PC 60000000 //PCとのシリアル速度（モニタリング表示用）
#define SPI_CLOCK 6000000// SPI通信の速度（6000000kHz推奨）
#define BAUDRATE 1250000 //ICSサーボの通信速度1.25M
#define TIMEOUT 2 //ICS返信待ちのタイムアウト時間。通信できてないか確認する場合には1000ぐらいに設定するとよい

/*
  -------------------------------------------------------------------------
  -- [PIN_ASSIGNs] Teensy4.0 ピンアサイン (TS-2-PIN) ------------------------
  -------------------------------------------------------------------------
  [GND]               -> GND
  [00] RX1, CRX2      -> ICS_3rd_TX
  [01] TX1, CTX2      -> ICS_3rd_RX
  [02]                -> LED（lights up when the processing time is not within the specified time.）
  [03]                -> (NeoPixel?)
  [04]                -> (NeoPixel?)
  [05]                -> ICS_Right_EN
  [06]                -> ICS_Left_EN
  [07] RX2            -> ICS_Left_TX
  [08] TX2            -> ICS_Left_RX
  [09]                -> SD_DAT3/CD (SD[2])
  [10] CS             -> SPI_CS (ESP32[15])
  [11] MOSI           -> SPI/SD_MOSI (ESP32[13]) & SD_CMD (SD[3])
  [12] MISO           -> SPI/SD_MISO (ESP32[12]) & SD_DAT0 (SD[7])
  [Vin]               -> 5V
  [AGND]              ->
  [3.3v]              -> GY-521(MPU6050) 3.3Vin & SD_VDD (SD[4])
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
  [13] SCK(CRX1)      -> SPI/SD_SCK (ESP32[14]) & SD_CLK (SD[5])

  -------------------------------------------------------------------------
  -- [HARD] サーボIDとロボット部位、軸との対応表 (TS-3-HARD) -------------------
  -------------------------------------------------------------------------
  ＜ICS_Left_Upper SIO1,SIO2＞
  ID    Parts-Axis
  [L01] 左肩ピッチ
  [L02] 左肩ロール
  [L03] 左肘ヨー
  [L04] 左肘ピッチ
  [L05] -
  ＜ICS_Left_Lower SIO3,SIO4＞
  ID    Parts-Axis
  [L00] 腰ヨー
  [L06] 左股ロール
  [L07] 左股ピッチ
  [L08] 左膝ピッチ
  [L09] 左足首ピッチ
  [L10] 左足首ロール
  ＜ICS_Right_Upper SIO5,SIO6＞
  ID    Parts-Axis
  [R00] 頭ヨー
  [R01] 右肩ピッチ
  [R02] 右肩ロール
  [R03] 右肘ヨー
  [R04] 右肘ピッチ
  [R05] -
  ＜ICS_Right_Lower SIO7,SIO8＞
  ID    Parts-Axis
  [R06] 右股ロール
  [R07] 右股ピッチ
  [R08] 右膝ピッチ
  [R09] 右足首ピッチ
  [R10] 右足首ロール

  -------------------------------------------------------------------------
  -- [MERIDIM] Meridim配列 一覧表 (TS-4-MERIDIM) ---------------------------
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
  [88]      ERROR CODE(bit 9-15), CLOCK(bit 8), DATA(bit 0-7)
  [89]      チェックサム
*/

//---------------------------------------------------
// [LIBRARY] ライブラリ関連 (TS-5-LIB) ----------------
//---------------------------------------------------

#include <Wire.h> //MPU-6050のライブラリ導入
#include <SPI.h> //SDカード用のライブラリ導入
#include <SD.h> //SDカード用のライブラリ導入
#include <TsyDMASPI.h> //SPI Master用のライブラリを導入
//#include <MadgwickAHRS.h> //MPU6050のライブラリ導入
#include <MPU6050_6Axis_MotionApps20.h> //MPU6050のライブラリ導入
#include <IcsHardSerialClass.h> //ICSサーボのライブラリ導入
#include <MsTimer2.h> //タイマーのライブラリ導入


//---------------------------------------------------
// [VARIABLE] 変数関連 (TS-6-VAL) --------------------
//---------------------------------------------------

// (TS-6-1) 変数一般
static const int MSG_BUFF = MSG_SIZE * 2; //Meridim配列の長さ（byte換算）
int checksum; //チェックサム計算用
int spi_ok = 0; //通信のエラーカウント
int spi_trial = 0; //通信のエラーカウント
bool file_open = 0; //SDカード用の変数
int k; //各サーボの計算用変数

// (TS-6-2) フラグ関連
bool flag_sensor_imu_writable = true;//メインが結果値を読み取る瞬間、サブスレッドによる書き込みをウェイト

// (TS-6-3) 共用体の宣言 : Meridim配列格納用、SPI送受信バッファ配列格納用
typedef union //共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
  short sval[MSG_SIZE + 4]; // short型で100個の配列データを持つ
  uint8_t bval[MSG_BUFF + 4]; //1バイト単位で200個の配列データを持つ
} UnionData;
UnionData s_spi_meridim; //Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_spi_meridim; //Meridim配列データ(short型、センサや角度は100倍値)
UnionData s_spi_meridim_dma; //SPI送信用の共用体のインスタンスを作成
UnionData r_spi_meridim_dma; //SPI受信用の共用体のインスタンスを作成

// (TS-6-4) タイマー管理用の変数
long frame_ms = FRAME_DURATION;// 1フレームあたりの単位時間(ms)
long merc = (long)millis(); // フレーム管理時計の時刻 Meridian Clock.
long curr = (long)millis(); // 現在時刻を取得
long curr_micro = (long)micros(); // 現在時刻を取得
int frame_count = 0;//サイン計算用の変数
int frame_count_diff = 2;//サインカーブ動作などのフレームカウントをいくつずつ進めるか
int joypad_frame_count = 0;//JOYPADのデータを読みに行くためのフレームカウント
char frame_sync_s = 0;//フレーム毎に0-199をカウントし、送信用Meridm[88]の下位8ビットに格納
char frame_sync_r_expect = 0;//フレーム毎に0-199をカウントし、受信値と比較
char frame_sync_r_resv = 0;//今フレームに受信したframe_sync_rを格納

// (TS-6-5) エラーカウント用
int err_esp_pc = 0;//PCの受信エラー（ESP32からのUDP）
int err_pc_esp = 0;//ESP32の受信エラー（PCからのUDP）
int err_esp_tsy = 0;//Teensyの受信エラー（ESP32からのSPI）
int err_tsy_esp = 0;//ESP32の受信エラー（TeensyからのSPI）
int err_esp_skip = 0;//UDP→ESP受信のカウントの連番スキップ回数
int err_tsy_skip = 0;//ESP→Teensy受信のカウントの連番スキップ回数
int err_pc_skip = 0; //PC受信のカウントの連番スキップ回数

// (TS-6-6) 各種モード設定
bool trim_adjust = TRIM_ADJUST_MODE; //トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
bool monitor_src = MONITOR_SRC; //Teensyでのシリアル表示:送信ソースデータ
bool monitor_send = MONITOR_SEND; //Teensyでのシリアル表示:送信データ
bool monitor_resv = MONITOR_RESV; //Teensyでのシリアル表示:受信データ
bool monitor_resv_short = MONITOR_RESV_SHORT; //Teensyでのシリアル表示:受信データ(short)
bool monitor_resv_check = MONITOR_RESV_CHECK; //Teensyでのシリアル表示:受信成功の可否
bool monitor_resv_error = MONITOR_RESV_ERROR; //Teensyでのシリアル表示:受信エラー率
bool monitor_all_error = MONITOR_ALL_ERROR; //Teensyでのシリアル表示:全経路の受信エラー率
bool monitor_rpy = MONITOR_RPY; //Teensyでのシリアル表示:IMUからのrpy換算値
bool monitor_joypad = MONITOR_JOYPAD; //Teensyでのシリアル表示:リモコンのデータ

// (TS-6-7) シリアル経由リモコンの受信用変数
unsigned short button_1 = 0;//受信ボタンデータ1群
unsigned short button_2 = 0;//受信ボタンデータ2群
short stick_Lx = 0;//受信ジョイスティックデータLx
short stick_Ly = 0;//受信ジョイスティックデータLy
short stick_Rx = 0;//受信ジョイスティックデータRx
short stick_Ry = 0;//受信ジョイスティックデータRy
unsigned short pad_btn = 0;//ボタン変数一般化変換

// (TS-6-8) MPU6050のアドレス、レジスタ設定値
#define I2C_CLOCK 400000// I2Cの速度（400kHz推奨）
#define MPU_STOCK 4//MPUで移動平均を取る際の元にする時系列データの個数
MPU6050 mpu;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
float ROLL, PITCH, YAW, YAW_ZERO;
float mpu_read[16] ;//mpuからの読み込んだ一次データacc_x,y,z,gyro_x,y,z,mag_x,y,z,gr_x,y,z,rpy_r,p,y,temp
float mpu_zeros[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //リセット用
float mpu_ave_data[16] ;//上記の移動平均値を入れる
float mpu_result[16] ;//加工後の最新のmpuデータ（二次データ）
float mpu_stock_data[MPU_STOCK][16] ;//上記の移動平均値計算用のデータストック
int mpu_stock_count = 0; //上記の移動平均値計算用のデータストックを輪番させる時の変数

VectorInt16 aa;         // [x, y, z]            加速度センサの測定値
VectorInt16 gyro;       // [x, y, z]            角速度センサの測定値
VectorInt16 mag;        // [x, y, z]            磁力センサの測定値
long temperature;         // センサの温度測定値

// (TS-6-9) サーボ設定関連
// (TS-6-9-1) ICSサーボのインスタンス設定
IcsHardSerialClass krs_L(&Serial2, EN_L_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_R(&Serial3, EN_R_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_3(&Serial1, EN_3_PIN, BAUDRATE, TIMEOUT);//3系もICSの場合

// (TS-6-9-2) KRSサーボのポジション用配列.degreeではなくサーボ値が入る
int s_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int s_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int r_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int r_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素

// (TS-6-9-3) 各サーボの各サーボのマウント判定用配列
bool idl_mt[15];//idlのありなし（mtはmountの略）
bool idr_mt[15];//idrのありなし（mtはmountの略）

// (TS-6-9-4) 各サーボの内外回転プラマイ方向補正用配列
float idl_pn[15];//idlのプラマイ（pnはposi/negaの略）
float idr_pn[15];//idrのプラマイ（pnはposi/negaの略）

// (TS-6-9-5-KRS) 各サーボの直立デフォルト値
int idl_n[15];//idlのニュートラル補正値（nはneutralの略）
int idr_n[15];//idrのニュートラル補正値（nはneutralの略）

// (TS-6-9-6) 各サーボのポジション値.中央値を0とした時の増減合計値
float idl_d[15];//idlの増減分（d）
float idr_d[15];//idrの増減分（d）

// SDカードテスト用
File myFile;


void setup() {
  //-------------------------------------------------------------------------
  //---- (TS-10) サーボ設定  -------------------------------------------------
  //-------------------------------------------------------------------------
  // (TS-10-1) 各サーボのマウントありなし（1:サーボあり、0:サーボなし）
  idl_mt[0]  = 1; //頭ヨー
  idl_mt[1]  = 1; //左肩ピッチ
  idl_mt[2]  = 1; //左肩ロール
  idl_mt[3]  = 1; //左肘ヨー
  idl_mt[4]  = 1; //左肘ピッチ
  idl_mt[5]  = 0; //左股ヨー
  idl_mt[6]  = 0; //左股ロール
  idl_mt[7]  = 0; //左股ピッチ
  idl_mt[8]  = 0; //左膝ピッチ
  idl_mt[9]  = 0; //左足首ピッチ
  idl_mt[10] = 0; //左足首ロール
  idl_mt[11] = 0; //追加テスト用
  idl_mt[12] = 0; //追加テスト用
  idl_mt[13] = 0; //追加テスト用
  idl_mt[14] = 0; //追加テスト用
  idr_mt[0]  = 1; //腰ヨー
  idr_mt[1]  = 1; //右肩ピッチ
  idr_mt[2]  = 1; //右肩ロール
  idr_mt[3]  = 0; //右肘ヨー
  idr_mt[4]  = 0; //右肘ピッチ
  idr_mt[5]  = 0; //右股ヨー
  idr_mt[6]  = 0; //右股ロール
  idr_mt[7]  = 0; //右股ピッチ
  idr_mt[8]  = 0; //右膝ピッチ
  idr_mt[9]  = 0; //右足首ピッチ
  idr_mt[10] = 0; //右足首ロール
  idr_mt[11] = 0; //追加テスト用
  idr_mt[12] = 0; //追加テスト用
  idr_mt[13] = 0; //追加テスト用
  idr_mt[14] = 0; //追加テスト用

  // (TS-10-2) 各サーボの内外回転プラマイ方向補正(1 or -1)
  idl_pn[0]  = 1;//頭ヨー
  idl_pn[1]  = 1;//左肩ピッチ
  idl_pn[2]  = 1;//左肩ロール
  idl_pn[3]  = 1;//左肘ヨー
  idl_pn[4]  = 1;//左肘ピッチ
  idl_pn[5]  = 1;//左股ヨー
  idl_pn[6]  = 1;//左股ロール
  idl_pn[7]  = 1;//左股ピッチ
  idl_pn[8]  = 1;//左膝ピッチ
  idl_pn[9]  = 1;//左足首ピッチ
  idl_pn[10] = 1;//左足首ロール
  idl_pn[11] = 1;//追加テスト用
  idl_pn[12] = 1;//追加テスト用
  idl_pn[13] = 1;//追加テスト用
  idl_pn[14] = 1;//追加テスト用
  idr_pn[0]  = 1;//腰ヨー
  idr_pn[1]  = 1;//右肩ピッチ
  idr_pn[2]  = 1;//右肩ロール
  idr_pn[3]  = 1;//右肘ヨー
  idr_pn[4]  = 1;//右肘ピッチ
  idr_pn[5]  = 1;//右股ヨー
  idr_pn[6]  = 1;//右股ロール
  idr_pn[7]  = 1;//右股ピッチ
  idr_pn[8]  = 1;//右膝ピッチ
  idr_pn[9]  = 1;//右足首ピッチ
  idr_pn[10] = 1;//右足首ロール
  idr_pn[11] = 1;//追加テスト用
  idr_pn[12] = 1;//追加テスト用
  idr_pn[13] = 1;//追加テスト用
  idr_pn[14] = 1;//追加テスト用

  // (TS-10-3) 各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375)
  //           直立状態になるよう、具体的な数値を入れて現物調整する
  idl_n[0]  = 0;//頭ヨー
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
  idr_n[0]  = 0; //腰ヨー
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

  //-------------------------------------------------------------------------
  //---- (TS-11) 起動時設定 --------------------------------------------------
  //-------------------------------------------------------------------------

  // (TS-11-1) 入出力ピンのモード設定
  pinMode(ERR_LED, OUTPUT);//通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）

  // (TS-11-2) シリアル設定
  Serial.begin(SERIAL_PC);//シリアルモニター表示
  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）

  // (TS-11-3) 起動時のインフォメーション表示表示(シリアルモニタ)
  Serial.println(VERSION);
  Serial.print("Set I2C speed:");
  Serial.println(I2C_CLOCK);
  Serial.print("Set SPI speed:");
  Serial.println(SPI_CLOCK);
  delay(500);

  // (TS-11-4) サーボ用シリアル設定
  krs_L.begin(); //サーボモータの通信初期設定。Serial2
  krs_R.begin(); //サーボモータの通信初期設定。Serial3
  krs_3.begin(); //サーボモータの通信初期設定。Serial1
  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）

  // (TS-11-5) I2CのSETUP
  if (IMU_MOUNT == 1) {
    setupMPU();
  }

  //SDカードの初期設定と書き込み
  Serial.print("Initializing SD card...");

  if (!SD.begin(CHIPSELECT_SD)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // open the file.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("Meridian Board read write test.");

    randomSeed(analogRead(8) * 10 + analogRead(7) * 2 + analogRead(6));
    int randNumber = random(1000, 9999); // 0から299の乱数を生成
    Serial.print("SD writing test ID :");
    Serial.println(randNumber);
    myFile.print("SD writing test ID :");
    myFile.println(randNumber);

    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("Opening test.txt...");

    // read from the file until there's nothing else in it:
    Serial.println("Reading texts in test.txt:");

    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  delay(100);

  
  // (TS-11-6) ESP32との通信用にSPI_MASTERを開始
  TsyDMASPI0.begin(SS, SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3));

  // (TS-11-7) SDカードの初期化

  // (TS-11-8) 配列のリセット
  memset(s_spi_meridim.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
  memset(r_spi_meridim.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
  memset(s_spi_meridim_dma.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
  memset(r_spi_meridim_dma.bval, 0, MSG_BUFF + 4); //配列要素を0でリセット
  memset(idl_d, 0, 15); //配列要素を0でリセット
  memset(idr_d, 0, 15); //配列要素を0でリセット

  // (TS-11-9) 割り込み処理系のセット
  // IMUのデータ取得
  MsTimer2::set(10, imu_getYawPitchRoll); // MPUの情報を取得 10msごとにチェック
  MsTimer2::start();


  // (TS-11-10) 変数の設定
  YAW_ZERO = 0;
  s_spi_meridim.sval[0] = MSG_SIZE ;//(マスターコマンド）

  // (TS-11-11) 起動時のディレイ用mercちょい足し(サーボ起動待ち用)
  merc = merc + 3000;



}


//-------------------------------------------------------------------------
//---- 計 算 系 の 関 数 各 種  ---------------------------------------------
//-------------------------------------------------------------------------

// ■ チェックサムの算出関数 --------------------------------------------------
short checksum_val(short arr[], int len) {
  int cksm = 0;
  for (int i = 0; i < len - 1; i++) {
    cksm += int(arr[i]);
  }
  //cksm = short(~cksm);
  return ~cksm;
}

// ■ チェックサムの判定関数 --------------------------------------------------
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

// ■ degreeをKRS値に変換 ----------------------------------------------------
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

// ■ KRSをdegree値に変換 -----------------------------------------------------
float Krs2Deg(int krs, float n, float pn) { //KRS値のほか idl_n[i], idl_pn[i] を入れる(右の場合はidr系)
  float x = (krs - 7500 - n) * 3.375 * pn;//新
  x = x / 100; //小数点以下2桁で取得する　
  return x;
}

// ■ floatを100倍してshortに収める 限界を超えたら限界値張り付き short型で返す-------
short float2HFshort(float val) {// float to Hundredfold short
  double x = val * 100; //floatの小数点以下を四捨五入して整数化
  if (x > 32766) {
    x = 32767;
  } else if (x < -32766) {
    x = -32767;
  }
  int y = (int)round(x);
  return (short)y;
}

// ■ IMUの初期設定 ------------------------------------------------------------
void setupMPU() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK); // 400kHz I2C clock. Comment this line if having compilation difficulties
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

// ■ IMUのDMP推定値取得 ----------------------------------------------------------
void imu_getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // 最新のIMU情報を取得
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //加速度の値
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu_read[0] = (float)aa.x;
    mpu_read[1] = (float)aa.y;
    mpu_read[2] = (float)aa.z;

    //ジャイロの値
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    mpu_read[3] = (float)gyro.x;
    mpu_read[4] = (float)gyro.y;
    mpu_read[5] = (float)gyro.z;

    //磁力センサの値
    mpu_read[6] = (float)mag.x;
    mpu_read[7] = (float)mag.y;
    mpu_read[8] = (float)mag.z;

    //重力DMP推定値
    mpu_read[9] = gravity.x;
    mpu_read[10] = gravity.y;
    mpu_read[11] = gravity.z;

    //相対方向DMP推定値
    mpu_read[12] = ypr[2] * 180 / M_PI;//DMP_ROLL推定値
    mpu_read[13] = ypr[1] * 180 / M_PI;//DMP_PITCH推定値
    mpu_read[14] = (ypr[0] * 180 / M_PI) - YAW_ZERO;//DMP_YAW推定値

    //温度
    mpu_read[15] = 0;//fifoBufferからの温度取得方法が今のところ不明。

    if (flag_sensor_imu_writable) {
      memcpy(mpu_result, mpu_read, sizeof(float) * 16);
    }

    if (monitor_rpy == 1) { //Teensyでのシリアル表示:IMUからのrpy換算値
      Serial.print("[Roll, Pitch, Yaw] ");
      Serial.print(mpu_read[12]);
      Serial.print(", ");
      Serial.print(mpu_read[13]);
      Serial.print(", ");
      Serial.println(mpu_read[14]);
    }
  }
}

void imu_moving_average() {//IMUデータの移動平均値の取得
  //二次元配列の0番から輪番で最新のデータを入れていく。指定個数を上回ったら0番に戻す
  mpu_stock_count ++;
  if (mpu_stock_count > MPU_STOCK) {
    mpu_stock_count = 0;
  }

  imu_getYawPitchRoll();//IMUから最新データを取得
  //配列の輪番該当箇所に最新のデータを上書きする
  memcpy(mpu_stock_data[mpu_stock_count], mpu_read, MPU_STOCK);
  for (int i = 0; i < 16; i++) {
    mpu_ave_data[i] = 0.0;
  }

  //値を合計していく
  for (int i = 0; i < MPU_STOCK; i++) {
    for (int j = 0; j < 16; j++) {
      mpu_ave_data[j] += mpu_stock_data[i][j];
    }
  }

  //値を割る
  for (int i = 0; i < 16; i++) {
    mpu_ave_data[i] = mpu_ave_data[i / MPU_STOCK];
  }
}


//-------------------------------------------------------------------------
//---- コ マ ン ド 系 の 関 数 各 種  ----------------------------------------
//-------------------------------------------------------------------------

// ■ サーボトリム調整 サーボオンで直立静止-------------------------------------
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

// ■ ヨー軸の原点リセット --------------------------------------------------------
void setyaw() {
  YAW_ZERO = ypr[0] * 180 / M_PI;
  s_spi_meridim.sval[0] = MSG_SIZE;
}

// ■ 全サーボオフ ---------------------------------------------------------------
void servo_all_off() {
  for (int h = 0; h < 5; h++) {
    for (int i = 0; i < 15; i++) {
      if (idl_mt[i] == 1) {
        krs_L.setFree(i);
      }
      if (idr_mt[i] == 1) {
        krs_R.setFree(i);
      }
      delayMicroseconds(2);
    }
  }
  delay(100);
  Serial.println("All servos off.");
}

// ■ JOYPAD処理 ---------------------------------------------------------------
void joypad_read() {
  if (JOYPAD_MOUNT == 2) {//KRR5FH(KRC-5FH)をICS_R系に接続している場合
    joypad_frame_count ++;
    if (joypad_frame_count >= JOYPAD_FRAME) {
      unsigned short buttonData;
      buttonData = krs_R.getKrrButton();
      delayMicroseconds(2);
      if (buttonData != KRR_BUTTON_FALSE) //ボタンデータが受信できていたら
      {
        button_1 = buttonData;
        if (monitor_joypad) {

          pad_btn = 0;
          if ((button_1 & 15) == 15) {//左側十時ボタン全部押しならstart押下とみなす
            pad_btn += 1;
          }
          else {
            //左側の十時ボタン
            pad_btn +=  (button_1 & 1) * 16 + ((button_1 & 2) >> 1) * 64 +  ((button_1 & 4) >> 2) * 32 + ((button_1 & 8) >> 3) * 128;
          }
          if ((button_1 & 368) == 368) pad_btn += 8;//右側十時ボタン全部押しならselect押下とみなす
          else {
            //右側十時ボタン
            pad_btn += ((button_1 & 16) >> 4) * 4096 + ((button_1 & 32) >> 5) * 16384 +  ((button_1 & 64) >> 6) * 8192 + ((button_1 & 256) >> 8) * 32768;
          }
          //L1,L2,R1,R2
          pad_btn += ((button_1 & 2048) >> 11) * 2048 + ((button_1 & 4096) >> 12) * 512 +  ((button_1 & 512) >> 9) * 1024 + ((button_1 & 1024) >> 10) * 256;
        }
      }
      joypad_frame_count = 0;
    }
  }
}


//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop() {

  //----  [ 1 ] E S P 3 2 と の S P I に よ る 送 受 信 処 理 -------------------------------

  // [1-1] ESP32とのSPI送受信の実行
  if (ESP32_MOUNT) {
    TsyDMASPI0.transfer(s_spi_meridim_dma.bval, r_spi_meridim_dma.bval, MSG_BUFF + 4);

    spi_trial ++;//SPI送受信回数のカウント


    // [1-2] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
    //チェックサムがOKならバッファから受信配列に転記
    if (checksum_rslt(r_spi_meridim_dma.sval, MSG_SIZE))
    {
      if (monitor_resv_check) {
        Serial.println("Rvok! ");//受信OKのシリアル表示
      }
      for (int i = 0; i < MSG_SIZE; i++) {
        r_spi_meridim.sval[i] = r_spi_meridim_dma.sval[i];
      }
      spi_ok ++;
      r_spi_meridim.bval[177] &= B11011111; //エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオフ
    } else
    {
      if (monitor_resv_check) {
        Serial.println("RvNG****");//受信NGのシリアル表示
        r_spi_meridim.bval[177] |= B00100000; //エラーフラグ13番(TeensyのESPからのSPI受信エラー検出)をオン
      }
    }

    // [1-3] 通信エラー処理(スキップ検出)
    frame_sync_r_expect ++;//フレームカウント予想値を加算
    if (frame_sync_r_expect > 199) { //予想値が200以上ならカウントを0に戻す
      frame_sync_r_expect = 0;
    }

    //Serial.print(int(frame_sync_r_resv));
    //Serial.print(":");
    //Serial.println(int(frame_sync_r_expect));

    //frame_sync_rの確認(受信フレームにスキップが生じていないかをMeridim[88]の下位8ビットのカウンターで判断)
    frame_sync_r_resv = r_spi_meridim.bval[176];//数値を受け取る

    if (frame_sync_r_resv == frame_sync_r_expect) //frame_sync_rの受信値が期待通りなら順番どおり受信
    {
      r_spi_meridim.bval[177] &= B11111101; //エラーフラグ9番(Teensy受信のスキップ検出)をオフ
      //Serial.println("Tsy sequensial OK!");
    } else
    {
      r_spi_meridim.bval[177] |= B00000010; //エラーフラグ9番(Teensy受信のスキップ検出)をオン
      if (frame_sync_r_resv == frame_sync_r_expect - 1)
      {
        frame_sync_r_expect = frame_sync_r_resv + 1; //同じ値を２回取得した場合には実際はシーケンスが進んだものとして補正（アルゴリズム要検討）
      } else
      {
        frame_sync_r_expect = frame_sync_r_resv; //取りこぼしについては現在の受信値を正解の予測値としてキープ
      }
      err_tsy_skip ++;
    }

    // [1-4] 通信エラー処理(エラーカウンタへの反映)
    if ((r_spi_meridim.bval[177] >> 7) & B00000001)//Meridim[88] bit15:PCのESP32からのUDP受信エラー
    {
      err_esp_pc ++;
    }
    if ((r_spi_meridim.bval[177] >> 6) & B00000001)//Meridim[88] bit14:ESP32のPCからのUDP受信エラー
    {
      err_pc_esp ++;
    }
    if ((r_spi_meridim.bval[177] >> 5) & B00000001)//Meridim[88] bit13:TeensyのESPからのSPI受信エラー
    {
      err_esp_tsy ++;
    }
    if ((r_spi_meridim.bval[177] >> 4) & B00000001)//Meridim[88] bit12:ESP32のTeensyからのSPI受信エラー
    {
      err_tsy_esp ++;
    }
    if ((r_spi_meridim.bval[177] >> 2) & B00000001)//Meridim[88] bit10:UDP→ESP受信のカウントのスキップ
    {
      err_esp_skip ++;
    }
    if ((r_spi_meridim.bval[177] >> 1) & B00000001)//Meridim[88] bit9:ESP→Teensy受信のカウントのスキップ
    {
      err_tsy_skip ++;
    }
    if ((r_spi_meridim.bval[177]) & B00000001)//Meridim[88] bit8:PC受信のカウントのスキップ
    {
      err_pc_skip ++;
    }

    //----  [ 2 ] シ リ ア ル モ ニ タ リ ン グ 表 示 処 理 2 -------------------------------

    // [2-1] //受信データの表示（SPI受信データShort型）
    if (monitor_resv) {
      Serial.print("  [Resv] ");
      for (int i = 0; i < MSG_SIZE; i++) {
        Serial.print(int (r_spi_meridim.sval[i]));
        Serial.print(",");
      }
      Serial.println();
    }

    // [2-2] //受信エラー率の表示
    if (monitor_resv_error) {
      if (spi_trial % 200 == 0) {
        Serial.print("error rate ");
        Serial.print(float(spi_trial - spi_ok) / float(spi_trial) * 100);
        Serial.print(" %  ");
        Serial.print(spi_trial - spi_ok);
        Serial.print("/");
        Serial.println(spi_trial);
      }
    }

    // [2-3] 全経路のエラー数の表示
    if (monitor_all_error) {
      Serial.print("[ERRORs] esp->pc:");
      Serial.print(err_esp_pc);
      Serial.print("  pc->esp:");
      Serial.print(err_pc_esp);
      Serial.print("  esp->tsy:");
      Serial.print(err_esp_tsy);
      Serial.print("  tsy->esp:");
      Serial.print(err_esp_tsy);
      Serial.print("  tsy-skip:");
      Serial.print(err_tsy_skip);//
      Serial.print("  esp-skip:");
      Serial.print(err_esp_skip);//
      Serial.print("  pc-skip:");
      Serial.print(err_pc_skip);//
      Serial.print("  seq:");
      Serial.print(int(frame_sync_r_resv));//
      Serial.print("  [ERR]:");
      Serial.print(r_spi_meridim.bval[177], BIN);
      Serial.println();
    }
  }

  //----  [ 3 ] 積 み 残 し 処 理  -------------------------------------------------
  //積み残しがあればここで処理

  //----  [ 4 ] 受 信 S P I デ ー タ を 送 信 S P I デ ー タ に 転 記  -----------------------
  memcpy(s_spi_meridim.bval, r_spi_meridim.bval, MSG_BUFF + 4);

  //----  [ 5 ] セ ン サ ー 類 読 み 取 り --------------------------------------------------
  //[5-1] IMUの値を取得
  //※IMUについてはタイマー割り込みで別途処理


  //----  [ 6 ] コ ン ト ロ ー ラ の 読 み 取 り  -------------------------------------------
  //[6-1] コントローラの値を取得
  if (JOYPAD_MOUNT == 1) {//SBDBTが接続設定されていれば受信チェック（未実装）
    Serial.print("SBDBT connection has not been programmed yet.");
  }
  if (JOYPAD_MOUNT == 2) {//KRC-5FH+KRR-5FHが接続設定されていれば受信チェック
    joypad_read();
    r_spi_meridim.sval[80] = pad_btn;
    s_spi_meridim.sval[80] = pad_btn;
  }


  //----  [ 7 ] Teensy 内 部 で 位 置 制 御 す る 場 合 の 処 理 -----------------------------

  // [7-1] マスターコマンドの判定によりこの工程の実行orスキップを分岐(デフォルトはMeridim配列数である90)

  //コマンド[90]: サーボオン 通常動作

  //コマンド[0]: 全サーボ脱力

  //コマンド[1]: サーボオン 通常動作

  //コマンド[2]: IMUのヨー軸設定(3-1-2)
  if (r_spi_meridim.sval[0] == UPDATE_YAW_CENTER) {
    setyaw();
  }

  //コマンド[3]: トリムモードがオンもしくはコマンド3の時はループ
  if ((trim_adjust == 1) or (r_spi_meridim.sval[0] == ENTER_TRIM_MODE)) {
    trimadjustment();
  }

  // [7-2] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i < 15; i++) {
    s_KRS_servo_pos_L[i] = Deg2Krs(float(r_spi_meridim.sval[i * 2 + 21]) / 100 * idl_pn[i], idl_n[i]); //
    s_KRS_servo_pos_R[i] = Deg2Krs(r_spi_meridim.sval[i * 2 + 51] / 100 * idr_pn[i], idr_n[i]); //
  }

  // [7-3] Teensyによる次回動作の計算

  // [7-4] センサーデータによる動作へのフィードバック加味

  // [7-5] 移動時間の決定

  // [7-6] Teensy内計算による次回動作をMeridim配列に書き込む


  //----  [ 8 ] サ ー ボ コ マ ン ド の 書 き 込 み ------------------------------------------

  // [8-1] Meridim配列をサーボ命令に変更

  // [8-2] サーボコマンドの配列に書き込み

  // [8-3] サーボデータのICS送信および返り値を取得
  //ICS_L系統の処理
  for (int i = 0; i < 3; i ++) {//接続したサーボの数だけ繰り返す。最大は15
    idl_d[i] = 0;
    if (idl_mt[i] == true) {
      if (r_spi_meridim.sval[(i * 2) + 20] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_L.setPos(i, s_KRS_servo_pos_L[i]);
      } else
      {
        k = krs_L.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
      }
      idl_d[i] = Krs2Deg(k, idl_n[i], idl_pn[i]);
    }
    delayMicroseconds(2);
  }
  //ICS_R系統の処理
  for (int i = 0; i < 3; i ++) {//接続したサーボの数だけ繰り返す。最大は15
    idr_d[i] = 0;
    if (idr_mt[i] == true) {
      if (r_spi_meridim.sval[(i * 2) + 50] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_R.setPos(i, s_KRS_servo_pos_R[i]);
      } else
      {
        k = krs_R.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得
      }
      idr_d[i] = Krs2Deg(k, idr_n[i], idr_pn[i]);
    }
    delayMicroseconds(2);
  }


  //ICS_3系統の処理
  for (int i = 0; i < 2; i ++) {//接続したサーボの数だけ繰り返す。L系統の3,4をICS3で処理する
    idl_d[i + 3] = 0;
    if (idl_mt[i + 3] == true) {
      if (r_spi_meridim.sval[((i + 3) * 2) + 20] == 1) { //受信配列のサーボコマンドが1ならPos指定
        k = krs_3.setPos(i, s_KRS_servo_pos_L[i + 3]);
      } else
      {
        k = krs_3.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
      }
      idl_d[i + 3] = Krs2Deg(k, idl_n[i + 3], idl_pn[i + 3]);
    }
    delayMicroseconds(2);
  }

  //----  [ 9 ] S P I 送 信 用 の Meridim 配 列 を 作 成 す る -------------------------------

  // [9-1] マスターコマンドを配列に格納
  //s_spi_meridim.sval[0] = MSG_SIZE ;//デフォルトのマスターコマンドは配列数

  // [9-2] 移動時間を配列に格納
  //s_spi_meridim.sval[1] = 10 ;//(移動時間）

  // [9-3] センサー値を配列に格納
  if (IMU_MOUNT == 1) {
    flag_sensor_imu_writable = false;
    s_spi_meridim.sval[2] = float2HFshort(mpu_result[0]);  //IMU_acc_x
    s_spi_meridim.sval[3] = float2HFshort(mpu_result[1]);  //IMU_acc_y
    s_spi_meridim.sval[4] = float2HFshort(mpu_result[2]);  //IMU_acc_z
    s_spi_meridim.sval[5] = float2HFshort(mpu_result[3]);  //IMU_gyro_x
    s_spi_meridim.sval[6] = float2HFshort(mpu_result[4]);  //IMU_gyro_y
    s_spi_meridim.sval[7] = float2HFshort(mpu_result[5]);  //IMU_gyro_z
    s_spi_meridim.sval[8] = float2HFshort(mpu_result[6]);  //IMU_mag_x
    s_spi_meridim.sval[9] = float2HFshort(mpu_result[7]);  //IMU_mag_y
    s_spi_meridim.sval[10] = float2HFshort(mpu_result[8]); //IMU_mag_z
    s_spi_meridim.sval[11] = float2HFshort(mpu_result[9]); //IMU_
    s_spi_meridim.sval[12] = float2HFshort(mpu_result[12]);//DMP_ROLL推定値
    s_spi_meridim.sval[13] = float2HFshort(mpu_result[13]);//DMP_PITCH推定値
    s_spi_meridim.sval[14] = float2HFshort(mpu_result[14]);//DMP_YAW推定値
    s_spi_meridim.sval[15] = float2HFshort(mpu_result[15]);//tempreature
    flag_sensor_imu_writable = true;
  }

  // [9-4] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i < 15; i++) {
    s_spi_meridim.sval[i * 2 + 20] = 0; //仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_spi_meridim.sval[i * 2 + 21] = float2HFshort(idl_d[i]); //仮にここでは最新のサーボ角度degreeを格納
  }
  for (int i = 0; i < 15; i++) {
    s_spi_meridim.sval[i * 2 + 50] = 0; //仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_spi_meridim.sval[i * 2 + 51] = float2HFshort(idr_d[i]); //仮にここでは最新のサーボ角度degreeを格納
  }

  // [9-5] フレームスキップ検出用のカウントをカウントアップして送信用に格納
  frame_sync_s ++;
  if (frame_sync_s > 199) {
    frame_sync_s = 0;
  }
  s_spi_meridim.bval[176] = frame_sync_s;

  // [9-6] カスタムデータを配列格納

  // [9-7] チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = checksum_val(s_spi_meridim.sval, MSG_SIZE);

  // [9-8] 送信データのSPIバッファへのバイト型書き込み
  for (int i = 0; i < MSG_BUFF; i++) {
    s_spi_meridim_dma.bval[i] = s_spi_meridim.bval[i];
  }


  //----  [ 10 ] シ リ ア ル モ ニ タ リ ン グ 表 示 処 理 1 -------------------------------

  // [10-1] シリアルモニタ表示（SPI送信データShort型）
  if (monitor_src == 1) {
    Serial.print("   [Src] ");
    for (int i = 0; i < MSG_SIZE; i++) {
      Serial.print(int (s_spi_meridim.sval[i]));
      Serial.print(",");
    }
    Serial.println();
  }
  // [10-1] シリアルモニタ表示（SPI送信データChar型）
  if (monitor_send == 1) {
    Serial.print("  [Send] ");
    for (int i = 0; i < MSG_BUFF; i++) {
      Serial.print(int (s_spi_meridim_dma.bval[i]));
      Serial.print(",");
    }
    Serial.println();
  }


  //----  [ 11 ] フ レ ー ム 終 端 処 理 ----------------------------------------------

  // [11-1] この時点で１フレーム内に処理が収まっていない時の処理
  curr = (long)millis(); // 現在時刻を更新
  if (curr > merc) { // 現在時刻がフレーム管理時計を超えていたらアラートを出す
    //シリアルに遅延msを表示
    Serial.println("*** processing delay :");
    Serial.println(curr - merc);
    digitalWrite(ERR_LED, HIGH);//処理落ちが発生していたらLEDを点灯
  }
  else {
    digitalWrite(ERR_LED, LOW);//処理が収まっていればLEDを消灯
  }

  // [11-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
  curr = (long)millis();
  curr_micro = (long)micros(); // 現在時刻を取得
  //Serial.println(merc * 1000 - curr_micro); //詳細な残り時間をμ秒単位でシリアル表示
  while (curr < merc) {
    curr = (long)millis();
  }

  // [11-3]フレーム管理時計mercのカウントアップ
  merc = merc + frame_ms;//フレーム管理時計を1フレーム分進める
  frame_count = frame_count + frame_count_diff;//サインカーブ動作用のフレームカウントをいくつずつ進めるかをここで設定。
}
