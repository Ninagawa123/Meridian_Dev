//FUTABA RS30x TTL SerialSetting for Teensy4.0 + Meridian Board Type.K ver1.2
//Meridian Board Type.K を用いてFUTABA製のRS303,RS304のサーボのID変更が可能です。
//
//参考)https://www.futaba.co.jp/product/robot/robot_download/sample_programs
//
//Pin Assign
//RX:Teensy pin0   :Meridian Board Type.K ではICS_3
//TX:Teensy pin1   :Meridian Board Type.K ではICS_3
//EN:Teensy pin23  :Meridian Board Type.K ではICS_3
//
//
//ICS_3にサーボを「１つだけ」接続してください。またICS_3は電力線が未接続ですので、サーボには別途5~7.4Vの電源を供給してください。
//ICS_1,ICS_2を利用する場合には「Serial1.」をそれぞれ「Serial2.」「Serial3.」に書き換えて実行してください。
//ICS_1,ICS_2の電源電圧はボードへの供給電圧そのままとなりますので、電源が12Vではなく7.4Vであることをご確認ください。
//
//実行すると設定した内容がシリアルモニタに表示されます。
//通信速度を変更した場合には、シリアル速度を再設定の改めてプログラムを書き込む必要があります。
//サーボのセッティング書き換え終了後に、新しく設定したIDのサーボを動作させます。中心値から前後に10度ずつゆっくりと動きます。

// ■■■■■■■■■■■■■■■ IDの設定&使い方（設定変更はここだけ変更して書き込み→実行すればOK） ■■■■■■■■■■■■■■■
//サーボのシリアル速度のボーレート設定
#define BAUDRATE 230400 //デフォルトは115,200bps、Meridianの推奨は230,400bps

//変更前のID
#define OLD_ID 255 //変更前のID。255は全てのIDを対象とするので、サーボを一つだけ接続している場合は無変更でOK。

//変更後のID
#define NEW_ID 1 //変更後のID。この数値を変更して実行すれば設定できる。


// ■■■■■■■■■■■■■■■ その他の設定 ■■■■■■■■■■■■■■■
/// グローバル変数定義
int EN_L_PIN = 23;                 // デジタルPin23を送信イネーブルピンに設定
int RS30x_speed = 100;//サーボ速度指定


// ■■■■■■■■■■■■■■■ RS30x R O M 書 き 込 み ■■■■■■■■■■■■■■■
void RS30x_RomWrite (unsigned char ID) {
  unsigned char RS30x_s_data[8];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

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
  for (int i = 2; i <= 6; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
  }
  //Serial.println();
  RS30x_s_data[7] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 7; i++) {
    Serial1.write(RS30x_s_data[i]);
  }
  delay(100);//ROM書込み時間待機。
  Serial1.flush();                 // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
  delay(100);

  Serial.print("Write data to ROM to ");
  if (ID == 255) {
    Serial.print("ALL Connected");
  } else {
    Serial.print(ID, DEC);
  }
  Serial.println(" RS30x servo's.");

}

// ■■■■■■■■■■■■■■■ RS30x リ ブ ー ト ■■■■■■■■■■■■■■■
void RS30x_Reboot (unsigned char ID) {
  unsigned char RS30x_s_data[8];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

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
  for (int i = 2; i <= 6; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
  }
  //Serial.println();
  RS30x_s_data[7] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 7; i++) {
    Serial1.write(RS30x_s_data[i]);
  }
  Serial1.flush();                 // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止

  Serial.print("Reboot ");
  if (ID == 255) {
    Serial.print("ALL Connected");
  } else {
    Serial.print(ID, DEC);
  }
  Serial.println(" RS30x servo's.");
  delay(100);                     //ROM書込み時間待機。
}

// ■■■■■■■■■■■■■■■ RS30x IDの変更 ■■■■■■■■■■■■■■■
void RS30x_ID_change (unsigned char ID, unsigned char dat) {
  unsigned char RS30x_s_data[9];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

  // 再起動書き込みパケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x00;          // Flags
  RS30x_s_data[4] = 0x04;          // Address
  RS30x_s_data[5] = 0x01;          // Length
  RS30x_s_data[6] = 0x01;          // Count
  RS30x_s_data[7] = dat;          // dat

  // チェックサム計算
  for (int i = 2; i <= 7; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
  }
  RS30x_s_data[8] = RS30x_s_cksum; // Sum

  // パケットデータ送信
  digitalWrite(EN_L_PIN, HIGH);    // 送信許可
  for (int i = 0; i <= 8; i++) {
    Serial1.write(RS30x_s_data[i]);
  }
  Serial1.flush();                 // データ送信完了待ち
  digitalWrite(EN_L_PIN, LOW);     // 送信禁止
  delay(100);

  Serial.print("Set ID ");
  if (ID == 255) {
    Serial.print("ALL Connected");
  } else {
    Serial.print(ID, DEC);
  }
  Serial.print(" RS30x servo's ID to ");
  Serial.print(dat);
  Serial.println(".");
}

// ■■■■■■■■■■■■■■■ RS30x サ ー ボ ト ル ク 設 定 ■■■■■■■■■■■■■■■
void RS30x_Torque (unsigned char ID, unsigned char dat) {
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
  RS30x_s_data[7] = dat;          // dat

  // チェックサム計算
  RS30x_s_cksum = 0;
  for (int i = 2; i <= 7; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i]; // ID～datまでのXOR
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


// ■■■■■■■■■■■■■■■ RS30x サ ー ボ 角 度 ・ 速 度 指 定 ■■■■■■■■■■■■■■■
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
  RS30x_s_cksum = 0;
  for (int i = 2; i <= 10; i++) {
    RS30x_s_cksum = RS30x_s_cksum ^ RS30x_s_data[i];  // ID～datまでのXOR
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
// サーボの各種セッティングの設定、実行と反映
void setup() {
  pinMode(EN_L_PIN, OUTPUT);  // デジタルPin2(EN_L_PIN)を出力に設定
  Serial.begin(60000000);     // Teensy4.0とPCとのシリアル通信速度
  Serial1.begin(BAUDRATE);      // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
  delay(1000);
  Serial.println("RS303,304 servomotor's ID setting start.");

  //■ IDの書き換え
  RS30x_ID_change (OLD_ID, NEW_ID) ;

  //■ ROM書き込みと再起動
  RS30x_RomWrite(255);//ROM書き込み
  RS30x_Reboot(255);//サーボ再起動

  //■ 全サーボトルクオン
  RS30x_Torque(255, 0x01);      //  ID = 255(255は全サーボ対象,ID1ならば0x01を指定、ID15ならば0x10を指定）, RS30x_Torque = ON   (0x01)
  delay(10);
}

// ■■■■■■■■■■■■■■■ M A I N ■■■■■■■■■■■■■■■
// サーボの各種セッティングの設定、実行と反映
void loop() {
  RS30x_Move(255, 100, RS30x_speed); // ID=255は全サーボ , GoalPosition = 10.0deg(100) , Time = 1.0sec(RS30x_speed=100)
  delay(1000);
  RS30x_Move(255, 60, 20); // ID=255は全サーボ , GoalPosition = 6.0deg(60) , Time = 0.2sec(20)
  delay(500);
  RS30x_Move(255, 100, 20); // ID=255は全サーボ , GoalPosition = 10.0deg(100) , Time = 0.2sec(20)
  delay(500);
  RS30x_Move(255, -100, RS30x_speed); // ID=255は全サーボ , GoalPosition = -10.0deg(-100) , Time = 1.0sec(RS30x_speed=100)
  delay(1000);
}
