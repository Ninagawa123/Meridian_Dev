//FUTABA RS30x TTL SerialSpeedChanger for Teensy4.0
//Pin Assign
//RX:Teensy pin0   :Meridian Board Type.K ではICS_3
//TX:Teensy pin1   :Meridian Board Type.K ではICS_3
//EN:Teensy pin23  :Meridian Board Type.K ではICS_3

/* グローバル変数定義 */
int EN_L_PIN = 23;                 // デジタルPin23を送信イネーブルピンに設定

// ■■■■■■■■■■■■■■■ 通 信 速 度 設 定 ■■■■■■■■■■■■■■■
void RS30x_SerialSpeed (unsigned char ID, unsigned char dat) {
  unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

  // パケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x00;          // Flags
  RS30x_s_data[4] = 0x06;          // Address
  RS30x_s_data[5] = 0x01;          // Length
  RS30x_s_data[6] = 0x01;          // Count
  RS30x_s_data[7] = dat;           // Data

  // チェックサム計算
  Serial.print("cksum_culc:");
  for (int i = 2; i <= 7; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～DATAまでのXOR
    Serial.print(RS30x_s_cksum, HEX);
    Serial.print(",");
  }
  Serial.println();
  RS30x_s_data[8] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  Serial.print("packetdata:");
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 8; i++) {
    Serial1.write(RS30x_s_data[i]);
    Serial.print(RS30x_s_data[i], HEX);
    Serial.print(",");
  }
  Serial.println();
  Serial1.flush();                 // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
  delay(1000);                     //待機

  // ROM書き込みパケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x40;          // Flags
  RS30x_s_data[4] = 0xFF;          // Address
  RS30x_s_data[5] = 0x00;          // Length
  RS30x_s_data[6] = 0x00;          // Count

  // チェックサム計算
  RS30x_s_cksum = 0;
  Serial.print("cksum_culc:");
  for (int i = 2; i <= 6; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～DATAまでのXOR
    Serial.print(RS30x_s_cksum, HEX);
    Serial.print(",");
  }
  Serial.println();
  RS30x_s_data[7] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  Serial.print("packetdata:");
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 7; i++) {
    Serial1.write(RS30x_s_data[i]);
    Serial.print(RS30x_s_data[i], HEX);
    Serial.print(",");
  }
  Serial1.flush();                 // データ送信完了待ち
  Serial.println("");              // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
  delay(1000);                     //ROM書込み時間待機。

  // 再起動書き込みパケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x20;          // Flags
  RS30x_s_data[4] = 0xFF;          // Address
  RS30x_s_data[5] = 0x00;          // Length
  RS30x_s_data[6] = 0x00;          // Count

  // チェックサム計算
  RS30x_s_cksum = 0;
  Serial.print("cksum_culc:");
  for (int i = 2; i <= 6; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～DATAまでのXOR
    Serial.print(RS30x_s_cksum, HEX);
    Serial.print(",");
  }
  Serial.println();
  RS30x_s_data[7] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  Serial.print("packetdata:");
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 7; i++) {
    Serial1.write(RS30x_s_data[i]);
    Serial.print(RS30x_s_data[i], HEX);
    Serial.print(",");
  }
  delay(100);                     //ROM書込み時間待機。
  Serial1.flush();                 // データ送信完了待ち
  Serial.println();
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
  delay(1000);                     //ROM書込み時間待機。
}


// ■■■■■■■■■■■■■■■ サ ー ボ ト ル ク 設 定 ■■■■■■■■■■■■■■■
void RS30x_Torque (unsigned char ID, unsigned char data) {
  unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

  // パケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x00;          // Flags
  RS30x_s_data[4] = 0x24;          // Address
  RS30x_s_data[5] = 0x01;          // Length
  RS30x_s_data[6] = 0x01;          // Count
  RS30x_s_data[7] = data;          // Data
  // チェックサム計算
  for (int i = 2; i <= 7; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～DATAまでのXOR
  }
  RS30x_s_data[8] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 8; i++) {
    Serial1.write(RS30x_s_data[i]);
  }
  Serial1.flush();                 // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
}


// ■■■■■■■■■■■■■■■ サ ー ボ 角 度 ・ 速 度 指 定 ■■■■■■■■■■■■■■■
void RS30x_Move (unsigned char ID, int Angle, int Speed) {
  unsigned char RS30x_s_data[12];  // 送信データバッファ [12byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

  // パケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x00;          // Flags
  RS30x_s_data[4] = 0x1E;          // Address
  RS30x_s_data[5] = 0x04;          // Length
  RS30x_s_data[6] = 0x01;          // Count
  // Angle
  RS30x_s_data[7] = (unsigned char)0x00FF & Angle;           // Low byte
  RS30x_s_data[8] = (unsigned char)0x00FF & (Angle >> 8);    // Hi  byte
  // Speed
  RS30x_s_data[9] = (unsigned char)0x00FF & Speed;           // Low byte
  RS30x_s_data[10] = (unsigned char)0x00FF & (Speed >> 8);   // Hi  byte
  // チェックサム計算
  for (int i = 2; i <= 10; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i];  // ID～DATAまでのXOR
  }
  RS30x_s_data[11] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  digitalWrite(EN_L_PIN, HIGH);     // 送信許可
  for (int i = 0; i <= 11; i++) {
    Serial1.write(RS30x_s_data[i]);
  }
  Serial1.flush();                  // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);      // 送信禁止
}


// ■■■■■■■■■■■■■■■ S E T U P ■■■■■■■■■■■■■■■
void setup() {
  pinMode(EN_L_PIN, OUTPUT);  // デジタルPin2(EN_L_PIN)を出力に設定
  Serial.begin(115200);       // シリアルモニタのボーレート 115,200bps
  Serial1.begin(115200);      // 元のボーレート 
  //Serial1.begin(230400);    // 元のボーレート　//ボーレート変更後にこのボーレートにて開始するとサーボがループで動く
  delay(1000);

  Serial.println("Command transmission start.");
  RS30x_SerialSpeed(255, 0x07); // 変更後のボーレート(サーボID(255は全サーボ),ボーレート番号（例は115,200）)
  // 0x00:9600, 0x01:14,400, 0x02:19,200, 0x03:28.800, 0x04:38,400,
  // 0x05:57,600, 0x06:76,800, 0x07:115,200, 0x08:153,600, 0x09:230,400
  delay(1000);
  Serial.println("Servo's baud rate changed.");
  Serial.println("Please restart after change baud rate in Adruino IDE.");
  RS30x_Torque(255, 0x01);      // ID = 1(0x01) , RS30x_Torque = ON   (0x01)
}


// ■■■■■■■■■■■■■■■ M A I N ■■■■■■■■■■■■■■■
void loop() {
  int RS30x_speed = 100;//サーボ速度指定

  while (1) {//サーボ動作確認
    RS30x_Move(255, 600, RS30x_speed); // ID = 1 , GoalPosition = 30.0deg(300) , Time = 1.0sec(100)
    delay(1000);
    RS30x_Move(255, -600, RS30x_speed); // ID = 1 , GoalPosition = -30.0deg(300) , Time = 1.0sec(100)
    delay(1000);
  }
}
