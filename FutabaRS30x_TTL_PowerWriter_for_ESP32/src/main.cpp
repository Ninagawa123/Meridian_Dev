//とりあえず動く版キープ

// FUTABA RS30x TTL SerialSetting for ESP32DevkitC + Meridian Board -LITE- 2022.07.23
//参考)https://www.futaba.co.jp/product/robot/robot_download/sample_programs
//
// Pin Assign
// RX :ESP32 pin16 Serial2 :Meridian Board -LITE- ICS_R / ICS変換基板 TX
// TX :ESP32 pin17 Serial2 :Meridian Board -LITE- ICS_R / ICS変換基板 RX
// EN :ESP32 pin4  Serial2 :Meridian Board -LITE- ICS_R / ICS変換基板 EN
// 5V :ESP32 5V            :Meridian Board -LITE- 5V    / ICS変換基板 IOREF
// GND:ESP32 GND           :Meridian Board -LITE- GND   / ICS変換基板 GND

// TX :ESP32 pin17 Serial2 :FUTABA RS30x Servo SIGNAL(WHITE/RED&WING)
// 5V :ESP32 5V            :FUTABA RS30x Servo POWER (RED)
// GND:ESP32 5V            :FUTABA RS30x Servo GNE   (BLACK)

#include <Arduino.h>

// 設定したいボーレート
// 値の対応表：0x00:9600, 0x01:14,400, 0x02:19,200, 0x03:28.800, 0x04:38,400,
//           0x05:57,600, 0x06:76,800, 0x07:115,200, 0x08:153,600, 0x09:230,400
//           0x0A:460,800, 0x0B:691,200,

/* グローバル変数定義 */
int EN_R_PIN = 4;     // デジタルPin23を送信イネーブルピンに設定
int RS30x_speed = 50; //サーボ速度指定
int FutabaBaudRates[12] = {9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400, 460800, 691200};

void RS30x_SetSerialSpeedALL(unsigned char ID, unsigned char dat)
{
    Serial.println("in RS30x_SetSerialSpeedALL().");
    unsigned char RS30x_s_data[9] = {0xFA, 0xAF, ID, 0x00, 0x06, 0x01, 0x01, dat}; // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0;                                               // チェックサム計算用変数

    // パケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x00; // Flags
    RS30x_s_data[4] = 0x06; // Address
    RS30x_s_data[5] = 0x01; // Length
    RS30x_s_data[6] = 0x01; // Count
    RS30x_s_data[7] = dat;  // dat

    String tmp = "";
    switch (dat)
    {
    case 0x00:
        tmp = "9600";
        break;
    case 0x01:
        tmp = "14,400";
        break;
    case 0x02:
        tmp = "19,200";
        break;
    case 0x03:
        tmp = "28.800";
        break;
    case 0x04:
        tmp = "38,400";
        break;
    case 0x05:
        tmp = "57,600";
        break;
    case 0x06:
        tmp = "76,800";
        break;
    case 0x07:
        tmp = "115,200";
        break;
    case 0x08:
        tmp = "153,600";
        break;
    case 0x09:
        tmp = "230,400";
        break;
    case 0x0A:
        tmp = "460,800";
        break;
    case 0x0B:
        tmp = "691,200";
        break;
    default:
        tmp = "115,200";
    }

    Serial.print("Set ID ");
    if (ID == 255)
    {
        Serial.println("ALL Connected.");
    }
    else
    {
        Serial.println(ID, DEC);
    }
    Serial.print(" RS30x servo's BAUDRATE to ");
    Serial.println(tmp);

    for (int i = 0; i < 12; i++)
    {
        Serial.println("near Serial2.begin(FutabaBaudRates[i]).");

        Serial2.begin(FutabaBaudRates[i]); // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
        // Serial2.begin(FutabaBaudRates[i]); // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
        delay(100);

        // チェックサム計算
        for (int i = 2; i < 8; i++)
        {
            RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
        }
        RS30x_s_data[8] = RS30x_s_cksum; // Sum

        // パケットデータ送信
        digitalWrite(EN_R_PIN, HIGH); // 送信許可
        for (int i = 0; i <= 8; i++)
        {
            Serial2.write(RS30x_s_data[i]);
        }
        Serial.println("near Serial2.flush().");

        Serial2.flush();             // データ送信完了待ち
        digitalWrite(EN_R_PIN, LOW); // 送信禁止

        Serial.print(".");

        delay(400);
        Serial2.end();
        delay(100);
    }
    Serial.println();
}

// ■■■■■■■■■■■■■■■ RS30x 通 信 速 度 設 定 ■■■■■■■■■■■■■■■
void RS30x_SetSerialSpeed(unsigned char ID, unsigned char dat)
{
    unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // パケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x00; // Flags
    RS30x_s_data[4] = 0x06; // Address
    RS30x_s_data[5] = 0x01; // Length
    RS30x_s_data[6] = 0x01; // Count
    RS30x_s_data[7] = dat;  // dat

    // チェックサム計算
    for (int i = 2; i <= 7; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    RS30x_s_data[8] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 8; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止

    String tmp = "";
    switch (dat)
    {
    case 0x00:
        tmp = "9600";
        break;
    case 0x01:
        tmp = "14,400";
        break;
    case 0x02:
        tmp = "19,200";
        break;
    case 0x03:
        tmp = "28.800";
        break;
    case 0x04:
        tmp = "38,400";
        break;
    case 0x05:
        tmp = "57,600";
        break;
    case 0x06:
        tmp = "76,800";
        break;
    case 0x07:
        tmp = "115,200";
        break;
    case 0x08:
        tmp = "153,600";
        break;
    case 0x09:
        tmp = "230,400";
        break;
    default:
        tmp = "115,200";
    }
    Serial.print("Set ID ");
    if (ID == 255)
    {
        Serial.print("ALL Connected");
    }
    else
    {
        Serial.print(ID, DEC);
    }
    Serial.print(" RS30x servo's BAUDRATE to ");
    Serial.print(tmp);
    Serial.println(".");
    delay(10); //待機
}

void RS30x_SetSerialSpeedAll(unsigned char ID, unsigned char dat)
{
    unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // パケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x00; // Flags
    RS30x_s_data[4] = 0x06; // Address
    RS30x_s_data[5] = 0x01; // Length
    RS30x_s_data[6] = 0x01; // Count
    RS30x_s_data[7] = dat;  // dat

    // チェックサム計算
    for (int i = 2; i <= 7; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    RS30x_s_data[8] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 8; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止

    String tmp = "";
    switch (dat)
    {
    case 0x00:
        tmp = "9600";
        break;
    case 0x01:
        tmp = "14,400";
        break;
    case 0x02:
        tmp = "19,200";
        break;
    case 0x03:
        tmp = "28.800";
        break;
    case 0x04:
        tmp = "38,400";
        break;
    case 0x05:
        tmp = "57,600";
        break;
    case 0x06:
        tmp = "76,800";
        break;
    case 0x07:
        tmp = "115,200";
        break;
    case 0x08:
        tmp = "153,600";
        break;
    case 0x09:
        tmp = "230,400";
        break;
    default:
        tmp = "115,200";
    }
    Serial.print("Set ID ");
    if (ID == 255)
    {
        Serial.print("ALL Connected");
    }
    else
    {
        Serial.print(ID, DEC);
    }
    Serial.print(" RS30x servo's BAUDRATE to ");
    Serial.print(tmp);
    Serial.println(".");
    delay(10); //待機
}

// ■■■■■■■■■■■■■■■ RS30x R O M 書 き 込 み ■■■■■■■■■■■■■■■
void RS30x_RomWrite(unsigned char ID)
{
    unsigned char RS30x_s_data[8];   // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // ROM書き込みパケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x40; // Flags
    RS30x_s_data[4] = 0xFF; // Address
    RS30x_s_data[5] = 0x00; // Length
    RS30x_s_data[6] = 0x00; // Count

    // チェックサム計算
    RS30x_s_cksum = 0;
    for (int i = 2; i <= 6; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    // Serial.println();
    RS30x_s_data[7] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 7; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    delay(100);                  // ROM書込み時間待機。
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止
    delay(100);

    Serial.print("Write data to ROM to ");
    if (ID == 255)
    {
        Serial.print("ALL Connected");
    }
    else
    {
        Serial.print(ID, DEC);
    }
    Serial.println(" RS30x servo's.");
}

// ■■■■■■■■■■■■■■■ RS30x リ ブ ー ト ■■■■■■■■■■■■■■■
void RS30x_Reboot(unsigned char ID)
{
    unsigned char RS30x_s_data[8];   // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // 再起動書き込みパケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x20; // Flags
    RS30x_s_data[4] = 0xFF; // Address
    RS30x_s_data[5] = 0x00; // Length
    RS30x_s_data[6] = 0x00; // Count

    // チェックサム計算
    RS30x_s_cksum = 0;
    for (int i = 2; i <= 6; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    // Serial.println();
    RS30x_s_data[7] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 7; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止

    Serial.print("Reboot ");
    if (ID == 255)
    {
        Serial.print("ALL Connected");
    }
    else
    {
        Serial.print(ID, DEC);
    }
    Serial.println(" RS30x servo's.");
    delay(100); // ROM書込み時間待機。
}

// ■■■■■■■■■■■■■■■ RS30x サ ー ボ ト ル ク 設 定 ■■■■■■■■■■■■■■■
void RS30x_Torque(unsigned char ID, unsigned char dat)
{
    unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // パケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x00; // Flags
    RS30x_s_data[4] = 0x24; // Address
    RS30x_s_data[5] = 0x01; // Length
    RS30x_s_data[6] = 0x01; // Count
    RS30x_s_data[7] = dat;  // dat

    // チェックサム計算
    for (int i = 2; i <= 7; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    RS30x_s_data[8] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 8; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止
}

// ■■■■■■■■■■■■■■■ RS30x サ ー ボ 角 度 ・ 速 度 指 定 ■■■■■■■■■■■■■■■
void RS30x_Move(unsigned char ID, int Angle, int Speed)
{
    unsigned char RS30x_s_data[12];  // 送信データバッファ [12byte]
    unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

    // パケットデータ生成
    RS30x_s_data[0] = 0xFA; // Header
    RS30x_s_data[1] = 0xAF; // Header
    RS30x_s_data[2] = ID;   // ID
    RS30x_s_data[3] = 0x00; // Flags
    RS30x_s_data[4] = 0x1E; // Address
    RS30x_s_data[5] = 0x04; // Length
    RS30x_s_data[6] = 0x01; // Count
    // Angle
    RS30x_s_data[7] = (unsigned char)0x00FF & Angle;        // Low byte
    RS30x_s_data[8] = (unsigned char)0x00FF & (Angle >> 8); // Hi  byte
    // Speed
    RS30x_s_data[9] = (unsigned char)0x00FF & Speed;         // Low byte
    RS30x_s_data[10] = (unsigned char)0x00FF & (Speed >> 8); // Hi  byte
    // チェックサム計算
    for (int i = 2; i <= 10; i++)
    {
        RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
    }
    RS30x_s_data[11] = RS30x_s_cksum; // Sum

    // パケットデータ送信
    digitalWrite(EN_R_PIN, HIGH); // 送信許可
    for (int i = 0; i <= 11; i++)
    {
        Serial2.write(RS30x_s_data[i]);
    }
    Serial2.flush();             // データ送信完了待ち
    digitalWrite(EN_R_PIN, LOW); // 送信禁止
}

void setup()
{
    pinMode(EN_R_PIN, OUTPUT); // デジタルPin2(EN_R_PIN)を出力に設定

    Serial.begin(200000); // Teensy4.0とPCとのシリアル通信速度
    delay(1000);
    Serial.println();

    // 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 153600, 230400, 460800, 691200

    Serial2.begin(FutabaBaudRates[6]); // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）

    RS30x_SetSerialSpeed(255, 0x01); //ボーレート変更

    //■ ROM書き込みと再起動　※下記の２行を設定の際は必ず必要になります
    RS30x_RomWrite(255); //※※※※何らかの設定を変更する際はこの行をコメントアウト※※※※
    delay(1000);
    delay(1000);
    RS30x_Reboot(255); //※※※※何らかの設定を変更する際はこの行をコメントアウト※※※※
    delay(1000);
    delay(1000);

    Serial2.flush(); // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
    Serial2.end();   // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
    delay(500);
    Serial2.begin(FutabaBaudRates[1]); // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）

    //■ 全サーボトルクオン
    RS30x_Torque(255, 0x01); // ID = 1(0x01) , RS30x_Torque = ON   (0x01)
    delay(1000);
}

void loop()
{
    RS30x_Move(255, 0, RS30x_speed); // ID=255は全サーボ , GoalPosition = 0deg(100) , Time = 1.0sec(RS30x_speed=100)
    Serial.print("+,");
    delay(1000);
    Serial.print("0,");
    delay(1000);
    Serial.print("0,");
    delay(1000);
    RS30x_Move(255, 600, RS30x_speed); // ID=255は全サーボ , GoalPosition = 10.0deg(100) , Time = 1.0sec(RS30x_speed=100)
    Serial.print("+,");
    delay(1000);
    RS30x_Move(255, -600, RS30x_speed); // ID=255は全サーボ , GoalPosition = -10.0deg(-100) , Time = 1.0sec(RS30x_speed=100)
    Serial.print("-,");
    delay(1000);
}

