/*
  非同期書き込みの例はSTS3215で動作確認済み。サーボの出荷時速度単位は0.0146rpm、サーボ動作速度V=3400
  もし出荷時速度単位が0.732rpmの場合は、速度をV=68に変更し、遅延計算式はT=[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000
  
  SyncWritePosEx同等処理版 - 一括送信最適化
*/

#define PIN_EN 4

#include <Arduino.h>

// 基本型定義
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;

// 命令定義
#define INST_SYNC_WRITE 0x83    // 同期書き込み命令（SyncWritePosEx用）
#define INST_REG_WRITE 0x04     // 非同期書き込み命令
#define INST_REG_ACTION 0x05    // 非同期実行命令

// SMS/STSレジスタ定義
#define SMS_STS_ACC 41
#define SMS_STS_TORQUE_ENABLE 40

// 最大パケットサイズ定義
#define MAX_PACKET_SIZE 256

// グローバル変数
HardwareSerial* servoSerial = &Serial2;  // 使用するシリアルポート
u8 responseLevel = 0;  // 応答不要（送信専用モード）
unsigned long ioTimeout = 100;

// エンディアン変換関数（SMS/STS用：ビッグエンディアン）
void Host2SMS(u8 *DataL, u8* DataH, u16 Data) {
    *DataH = (Data >> 8);     // 上位バイト
    *DataL = (Data & 0xff);   // 下位バイト
}

// シリアル送信関数
void writeServo(u8 data) {
    servoSerial->write(data);
}

void writeServo(u8* data, int length) {
    servoSerial->write(data, length);
}

void flushServo() {
    servoSerial->flush();
    delay(5);  // 送信完了確保
}

// 受信バッファフラッシュ
void flushReceiveBuffer() {
    while (servoSerial->read() != -1);
}

// SyncWritePosEx実装（一括送信最適化版）
bool SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]) {
    // 入力チェック
    if (IDN == 0 || IDN > 20) {
        Serial.println("Error: Invalid servo count");
        return false;
    }
    
    // 各サーボのデータサイズ = 7バイト (ACC + Position + Time + Speed)
    const u8 dataPerServo = 7;
    const u8 messageLength = ((dataPerServo + 1) * IDN + 4); // +1はID, +4は基本ヘッダ
    
    // パケット全体のサイズ計算（ヘッダ7バイト + データ部 + チェックサム1バイト）
    const u16 totalPacketSize = 7 + (dataPerServo + 1) * IDN + 1;
    
    if (totalPacketSize > MAX_PACKET_SIZE) {
        Serial.println("Error: Packet size too large");
        return false;
    }
    
    Serial.print("SyncWrite: Preparing ");
    Serial.print(IDN);
    Serial.print(" servos, packet size: ");
    Serial.print(totalPacketSize);
    Serial.println(" bytes");
    
    // パケットバッファ準備
    u8 packet[MAX_PACKET_SIZE];
    u16 packetIndex = 0;
    
    // ヘッダ構築
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xfe;           // ブロードキャストID
    packet[packetIndex++] = messageLength;
    packet[packetIndex++] = INST_SYNC_WRITE;
    packet[packetIndex++] = SMS_STS_ACC;    // 開始レジスタアドレス
    packet[packetIndex++] = dataPerServo;   // 各サーボのデータ長
    
    // チェックサム計算開始
    u8 checksum = 0xfe + messageLength + INST_SYNC_WRITE + SMS_STS_ACC + dataPerServo;
    
    // 各サーボのIDとデータをパケットに追加
    for (u8 i = 0; i < IDN; i++) {
        s16 pos = Position[i];
        
        // 負の位置の処理（SMS/STS仕様）
        if (pos < 0) {
            pos = -pos;
            pos |= (1 << 15);  // 方向ビットを設定
        }
        
        // 速度とACCの取得（NULLチェック付き）
        u16 speed = Speed ? Speed[i] : 0;
        u8 acc = ACC ? ACC[i] : 0;
        
        // サーボIDを追加
        packet[packetIndex++] = ID[i];
        checksum += ID[i];
        
        // 7バイトのデータを追加
        packet[packetIndex++] = acc;  // 加速度
        checksum += acc;
        
        // 位置データ（2バイト）
        u8 posL, posH;
        Host2SMS(&posL, &posH, pos);
        packet[packetIndex++] = posL;
        packet[packetIndex++] = posH;
        checksum += posL + posH;
        
        // 時間データ（2バイト、0固定）
        packet[packetIndex++] = 0;
        packet[packetIndex++] = 0;
        // チェックサムには0を加算（変化なし）
        
        // 速度データ（2バイト）
        u8 speedL, speedH;
        Host2SMS(&speedL, &speedH, speed);
        packet[packetIndex++] = speedL;
        packet[packetIndex++] = speedH;
        checksum += speedL + speedH;
        
        Serial.print("  ID ");
        Serial.print(ID[i]);
        Serial.print(": Pos=");
        Serial.print(Position[i]);
        Serial.print(", Speed=");
        Serial.print(speed);
        Serial.print(", ACC=");
        Serial.println(acc);
    }
    
    // チェックサムを追加
    packet[packetIndex++] = ~checksum;
    
    // パケット全体を一括送信
    Serial.print("Sending packet: ");
    Serial.print(packetIndex);
    Serial.println(" bytes");
    
    writeServo(packet, packetIndex);
    flushServo();
    
    Serial.println("SyncWrite packet sent successfully");
    return true;
}

// トルクイネーブル（一括送信版）
bool EnableTorque(u8 ID, u8 Enable) {
    // パケットバッファ準備
    u8 packet[8];  // 最大8バイト
    u8 packetIndex = 0;
    u8 messageLength = 4;
    
    // ヘッダ構築
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = ID;
    packet[packetIndex++] = messageLength;
    packet[packetIndex++] = INST_REG_WRITE;
    packet[packetIndex++] = SMS_STS_TORQUE_ENABLE;
    
    // データ追加
    packet[packetIndex++] = Enable;
    
    // チェックサム計算と追加
    u8 checksum = ID + messageLength + INST_REG_WRITE + SMS_STS_TORQUE_ENABLE + Enable;
    packet[packetIndex++] = ~checksum;
    
    // 一括送信
    writeServo(packet, packetIndex);
    flushServo();
    
    return true;
}

// 汎用パケット構築・送信関数
bool sendPacket(u8 targetID, u8 instruction, u8 startAddress, u8* data, u8 dataLength) {
    // パケットサイズ計算
    u8 messageLength = dataLength ? (dataLength + 3) : 2;
    u8 totalSize = dataLength ? (6 + dataLength + 1) : (5 + 1);
    
    if (totalSize > MAX_PACKET_SIZE) {
        Serial.println("Error: Packet too large");
        return false;
    }
    
    // パケットバッファ準備
    u8 packet[MAX_PACKET_SIZE];
    u8 packetIndex = 0;
    
    // ヘッダ構築
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = targetID;
    packet[packetIndex++] = messageLength;
    packet[packetIndex++] = instruction;
    
    // アドレスとデータ（存在する場合）
    u8 checksum = targetID + messageLength + instruction;
    if (data && dataLength > 0) {
        packet[packetIndex++] = startAddress;
        checksum += startAddress;
        
        for (u8 i = 0; i < dataLength; i++) {
            packet[packetIndex++] = data[i];
            checksum += data[i];
        }
    }
    
    // チェックサム追加
    packet[packetIndex++] = ~checksum;
    
    // 一括送信
    writeServo(packet, packetIndex);
    flushServo();
    
    return true;
}

// 設定関数
void setServoSerial(HardwareSerial* serial) {
    servoSerial = serial;
}

void setIOTimeout(unsigned long timeout) {
    ioTimeout = timeout;
}

void setResponseLevel(u8 level) {
    responseLevel = level;
}

// サーボ設定
u8 armIDs[] = {1, 2, 3, 4, 5, 6};
s16 ServoPositions[] = {0, 0, 0, 0, 0, 0};
u16 ServoSpeeds[] = {0, 0, 0, 0, 0, 0};
u8 ServoACCs[] = {0, 0, 0, 0, 0, 0};

// 往復移動用変数
bool currentDirection = true;  // true: 4000へ, false: 500へ
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 2000;  // 2秒間隔

// 往復移動の実行（最適化版）
void executeReciprocationSync() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastMoveTime >= MOVE_INTERVAL) {
        // 目標位置を決定
        s16 targetPosition = currentDirection ? 4000 : 500;
        
        Serial.print("Moving to ");
        Serial.print(currentDirection ? "MAX" : "MIN");
        Serial.print(" position: ");
        Serial.println(targetPosition);
        
        // アクティブなサーボ（ID1,2）の設定
        u8 activeIDs[] = {1, 2};
        s16 activePositions[] = {targetPosition, targetPosition};
        u16 activeSpeeds[] = {1000, 1000};
        u8 activeACCs[] = {50, 50};
        
        // SyncWritePosExで一括送信（最適化版）
        SyncWritePosEx(activeIDs, 2, activePositions, activeSpeeds, activeACCs);
        
        // 方向切り替え
        currentDirection = !currentDirection;
        lastMoveTime = currentTime;
        
        Serial.print("Next move in ");
        Serial.print(MOVE_INTERVAL / 1000);
        Serial.println(" seconds");
        Serial.println();
    }
}

// パケット内容をHEXで表示するデバッグ関数
void debugPacketContent(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]) {
    Serial.println("\n=== Packet Debug ===");
    
    // パケット構築（デバッグ用）
    const u8 dataPerServo = 7;
    const u8 messageLength = ((dataPerServo + 1) * IDN + 4);
    const u16 totalPacketSize = 7 + (dataPerServo + 1) * IDN + 1;
    
    u8 packet[MAX_PACKET_SIZE];
    u16 packetIndex = 0;
    
    // ヘッダ
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xff;
    packet[packetIndex++] = 0xfe;
    packet[packetIndex++] = messageLength;
    packet[packetIndex++] = INST_SYNC_WRITE;
    packet[packetIndex++] = SMS_STS_ACC;
    packet[packetIndex++] = dataPerServo;
    
    u8 checksum = 0xfe + messageLength + INST_SYNC_WRITE + SMS_STS_ACC + dataPerServo;
    
    // データ部
    for (u8 i = 0; i < IDN; i++) {
        s16 pos = Position[i];
        if (pos < 0) {
            pos = -pos;
            pos |= (1 << 15);
        }
        
        u16 speed = Speed ? Speed[i] : 0;
        u8 acc = ACC ? ACC[i] : 0;
        
        packet[packetIndex++] = ID[i];
        checksum += ID[i];
        
        packet[packetIndex++] = acc;
        checksum += acc;
        
        u8 posL, posH;
        Host2SMS(&posL, &posH, pos);
        packet[packetIndex++] = posL;
        packet[packetIndex++] = posH;
        checksum += posL + posH;
        
        packet[packetIndex++] = 0;
        packet[packetIndex++] = 0;
        
        u8 speedL, speedH;
        Host2SMS(&speedL, &speedH, speed);
        packet[packetIndex++] = speedL;
        packet[packetIndex++] = speedH;
        checksum += speedL + speedH;
    }
    
    packet[packetIndex++] = ~checksum;
    
    // HEX表示
    Serial.print("Packet (");
    Serial.print(packetIndex);
    Serial.print(" bytes): ");
    for (u16 i = 0; i < packetIndex; i++) {
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("==================\n");
}

// 初期化
void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000);

    // ENピン設定（送信専用モード）
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, HIGH);

    delay(1000);
    
    Serial.println("=====================================");
    Serial.println("SMS/STS Servo SyncWrite Optimized");
    Serial.println("Batch Transmission Version");
    Serial.println("=====================================");
    Serial.println("Communication method: SyncWrite (0x83)");
    Serial.println("Optimization: Single writeServo() call");
    Serial.println("Packet size: Calculated dynamically");
    Serial.println("=====================================");

    // サーボのトルクイネーブル
    Serial.println("Enabling torque for active servos...");
    EnableTorque(1, 1);
    delay(100);
    EnableTorque(2, 1);
    delay(100);
    
    // 初期位置設定
    Serial.println("Setting initial positions...");
    u8 initIDs[] = {1, 2};
    s16 initPositions[] = {500, 500};
    u16 initSpeeds[] = {800, 800};
    u8 initACCs[] = {30, 30};
    
    // デバッグ表示
    debugPacketContent(initIDs, 2, initPositions, initSpeeds, initACCs);
    
    SyncWritePosEx(initIDs, 2, initPositions, initSpeeds, initACCs);
    delay(2000);  // 初期位置到達待ち
    
    Serial.println("Starting optimized SyncWrite motion...");
    Serial.println();
    
    lastMoveTime = millis();
}

void loop() {
    // メインの往復移動（最適化版）
    executeReciprocationSync();
    
    delay(50);  // CPU負荷軽減
}
