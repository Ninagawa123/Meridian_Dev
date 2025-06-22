/*
　ESP32+MeridainBoard , Serial2(RightSide)にて動作
  非同期書き込みの例はSTS3215で動作確認済み。サーボの出荷時速度単位は0.0146rpm、サーボ動作速度V=3400
  もし出荷時速度単位が0.732rpmの場合は、速度をV=68に変更し、遅延計算式はT=[(P1-P0)/(50*V)]*1000+[(50*V)/(A*100)]*1000
  SyncWritePosEx同等処理版 - 1つのパケットで複数サーボを制御
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

// SyncWritePosEx実装（1つのパケットで複数サーボ制御）
bool SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]) {
    // 入力チェック
    if (IDN == 0 || IDN > 20) {
        Serial.println("Error: Invalid servo count");
        return false;
    }
    
    // 各サーボのデータサイズ = 7バイト (ACC + Position + Time + Speed)
    const u8 dataPerServo = 7;
    const u8 messageLength = ((dataPerServo + 1) * IDN + 4); // +1はID, +4は基本ヘッダ
    
    Serial.print("SyncWrite: Sending ");
    Serial.print(IDN);
    Serial.println(" servos in one packet");
    
    // ヘッダ送信
    writeServo(0xff);
    writeServo(0xff);
    writeServo(0xfe);           // ブロードキャストID
    writeServo(messageLength);
    writeServo(INST_SYNC_WRITE);
    writeServo(SMS_STS_ACC);    // 開始レジスタアドレス
    writeServo(dataPerServo);   // 各サーボのデータ長
    
    // チェックサム計算開始
    u8 checksum = 0xfe + messageLength + INST_SYNC_WRITE + SMS_STS_ACC + dataPerServo;
    
    // 各サーボのIDとデータ送信
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
        
        // サーボIDを送信
        writeServo(ID[i]);
        checksum += ID[i];
        
        // 7バイトのデータを送信
        u8 dataBuffer[7];
        dataBuffer[0] = acc;  // 加速度
        Host2SMS(&dataBuffer[1], &dataBuffer[2], pos);    // 位置
        Host2SMS(&dataBuffer[3], &dataBuffer[4], 0);      // 時間（0固定）
        Host2SMS(&dataBuffer[5], &dataBuffer[6], speed);  // 速度
        
        // データ送信とチェックサム計算
        for (u8 j = 0; j < 7; j++) {
            writeServo(dataBuffer[j]);
            checksum += dataBuffer[j];
        }
        
        Serial.print("  ID ");
        Serial.print(ID[i]);
        Serial.print(": Pos=");
        Serial.print(Position[i]);
        Serial.print(", Speed=");
        Serial.print(speed);
        Serial.print(", ACC=");
        Serial.println(acc);
    }
    
    // チェックサム送信
    writeServo(~checksum);
    flushServo();
    
    Serial.println("SyncWrite packet sent successfully");
    return true;
}

// トルクイネーブル（個別送信）
bool EnableTorque(u8 ID, u8 Enable) {
    u8 messageLength = 4;
    u8 checksum = 0;
    
    // ヘッダ送信
    writeServo(0xff);
    writeServo(0xff);
    writeServo(ID);
    writeServo(messageLength);
    writeServo(INST_REG_WRITE);
    writeServo(SMS_STS_TORQUE_ENABLE);
    
    // データ送信
    writeServo(Enable);
    
    // チェックサム計算と送信
    checksum = ID + messageLength + INST_REG_WRITE + SMS_STS_TORQUE_ENABLE + Enable;
    writeServo(~checksum);
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

// 往復移動の実行（SyncWritePosEx版）
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
        
        // SyncWritePosExで一括送信（1つのパケット）
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

//// 全サーボ制御版（参考）
//void executeAllServosSync() {
//    static bool allDirection = true;
//    static unsigned long lastAllMove = 0;
//    const unsigned long ALL_MOVE_INTERVAL = 5000;  // 5秒間隔
//    
//    unsigned long currentTime = millis();
//    
//    if (currentTime - lastAllMove >= ALL_MOVE_INTERVAL) {
//        Serial.println("Moving all 6 servos with SyncWrite:");
//        
//        // 全6台のサーボ設定例
//        s16 allPositions[6];
//        u16 allSpeeds[] = {800, 900, 1000, 850, 950, 750};
//        u8 allACCs[] = {40, 45, 50, 42, 48, 38};
//        
//        if (allDirection) {
//            // MAX位置パターン
//            allPositions[0] = 4000; allPositions[1] = 4000; allPositions[2] = 3500;
//            allPositions[3] = 3800; allPositions[4] = 3600; allPositions[5] = 3700;
//        } else {
//            // MIN位置パターン  
//            allPositions[0] = 500;  allPositions[1] = 500;  allPositions[2] = 800;
//            allPositions[3] = 600;  allPositions[4] = 700;  allPositions[5] = 650;
//        }
//        
//        // 全サーボを1つのパケットで制御
//        SyncWritePosEx(armIDs, 6, allPositions, allSpeeds, allACCs);
//        
//        allDirection = !allDirection;
//        lastAllMove = currentTime;
//    }
//}

// 初期化
void setup() {
    Serial.begin(115200);
    Serial2.begin(1000000);

    // ENピン設定（送信専用モード）
    pinMode(PIN_EN, OUTPUT);
    digitalWrite(PIN_EN, HIGH);

    delay(1000);
    
    Serial.println("=====================================");
    Serial.println("SMS/STS Servo SyncWritePosEx Version");
    Serial.println("1 Packet = Multiple Servos Control");
    Serial.println("=====================================");
    Serial.println("Communication method: SyncWrite (0x83)");
    Serial.println("Servos per packet: Multiple");
    Serial.println("Sync accuracy: High (±few ms)");
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
    
    SyncWritePosEx(initIDs, 2, initPositions, initSpeeds, initACCs);
    delay(2000);  // 初期位置到達待ち
    
    Serial.println("Starting SyncWrite reciprocating motion...");
    Serial.println();
    
    lastMoveTime = millis();
}

void loop() {
    // メインの往復移動（ID1,2のみ）
    executeReciprocationSync();
    
    // 全サーボ制御のデモ（コメントアウト解除で有効）
    // executeAllServosSync();
    
    // シリアルコマンド処理
    handleSerialCommands();
    
    delay(50);  // CPU負荷軽減
}

// シリアルコマンド処理
void handleSerialCommands() {
    if (Serial.available()) {
        char command = Serial.read();
        
        switch (command) {
            case 'm':
            case 'M':
                Serial.println("Manual move triggered");
                lastMoveTime = 0;  // 強制的に次の移動をトリガー
                break;
                
            case 'a':
            case 'A':
                Serial.println("All servos demo move");
                executeAllServosSync();
                break;
                
            case 't':
            case 'T':
                Serial.println("Testing SyncWrite with different patterns:");
                testSyncWritePatterns();
                break;
                
            case 's':
            case 'S':
                Serial.println("=== Current Status ===");
                Serial.print("Direction: ");
                Serial.println(currentDirection ? "to 4000" : "to 500");
                Serial.print("Next move in: ");
                Serial.print((MOVE_INTERVAL - (millis() - lastMoveTime)) / 1000);
                Serial.println(" seconds");
                Serial.println("======================");
                break;
                
            case 'h':
            case 'H':
                Serial.println("=== Commands ===");
                Serial.println("M - Manual move");
                Serial.println("A - All servos demo");
                Serial.println("T - Test patterns");
                Serial.println("S - Show status");
                Serial.println("H - Show help");
                Serial.println("================");
                break;
        }
        
        // バッファクリア
        while (Serial.available()) Serial.read();
    }
}

// SyncWriteのテストパターン
void testSyncWritePatterns() {
    Serial.println("Pattern 1: Wave motion");
    u8 testIDs[] = {1, 2, 3, 4, 5, 6};
    s16 wavePositions[] = {1000, 2000, 3000, 4000, 3000, 2000};
    u16 waveSpeeds[] = {500, 600, 700, 800, 700, 600};
    u8 waveACCs[] = {30, 35, 40, 45, 40, 35};
    
    SyncWritePosEx(testIDs, 6, wavePositions, waveSpeeds, waveACCs);
    delay(3000);
    
    Serial.println("Pattern 2: Alternating motion");
    s16 altPositions[] = {4000, 500, 4000, 500, 4000, 500};
    u16 altSpeeds[] = {1000, 1000, 1000, 1000, 1000, 1000};
    u8 altACCs[] = {50, 50, 50, 50, 50, 50};
    
    SyncWritePosEx(testIDs, 6, altPositions, altSpeeds, altACCs);
    delay(3000);
    
    Serial.println("Pattern 3: Center return");
    s16 centerPositions[] = {2200, 2200, 2200, 2200, 2200, 2200};
    u16 centerSpeeds[] = {800, 800, 800, 800, 800, 800};
    u8 centerACCs[] = {40, 40, 40, 40, 40, 40};
    
    SyncWritePosEx(testIDs, 6, centerPositions, centerSpeeds, centerACCs);
    delay(2000);
    
    Serial.println("Test patterns completed");
}
