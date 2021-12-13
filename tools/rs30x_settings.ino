//FUTABA RS30x TTL SerialSetting for Teensy4.0
//Pin Assign
//RX:Teensy pin0   :Meridian Board Type.K ではICS_3
//TX:Teensy pin1   :Meridian Board Type.K ではICS_3
//EN:Teensy pin23  :Meridian Board Type.K ではICS_3

//Meridian Board Type.K を用いてFUTABA製のRS303,RS304のサーボの各種セッティング変更が可能です。
//現在、ボーレート、返信ディレイが変更可能です。

//ICS_3にサーボを接続してください。またICS_3は電力線が未接続ですので、サーボには別途5~7.4Vの電源を供給してください。
//ICS_1,ICS_2を利用する場合には「Serial1.」をそれぞれ「Serial2.」「Serial3.」に書き換えて実行してください。
//ICS_1,ICS_2の電源電圧はボードへの供給電圧そのままとなりますので、電源が12Vではなく7.4Vであることをご確認ください。

//実行すると設定した内容がシリアルモニタに表示されますのでご確認ください。
//通信速度を変更した場合には、シリアル速度を再設定の改めてプログラムを書き込む必要があります。
//サーボのセッティング書き換え終了後に、接続された全サーボが動作します。中心値から前後に10度だけゆっくりと動きます。


/* グローバル変数定義 */
int EN_L_PIN = 23;                 // デジタルPin23を送信イネーブルピンに設定
int RS30x_speed = 100;//サーボ速度指定

// ■■■■■■■■■■■■■■■ RS30x 通 信 速 度 設 定 ■■■■■■■■■■■■■■■
void RS30x_SetSerialSpeed (unsigned char ID, unsigned char dat) {
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
  RS30x_s_data[7] = dat;           // dat

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

  String tmp = "";
  switch (dat) {
    case 0x00:
      tmp = "9600"; break;
    case 0x01:
      tmp = "14,400"; break;
    case 0x02:
      tmp = "19,200"; break;
    case 0x03:
      tmp = "28.800"; break;
    case 0x04:
      tmp = "38,400"; break;
    case 0x05:
      tmp = "57,600"; break;
    case 0x06:
      tmp = "76,800"; break;
    case 0x07:
      tmp = "115,200"; break;
    case 0x08:
      tmp = "153,600"; break;
    case 0x09:
      tmp = "230,400"; break;
    default:
      tmp = "115,200";
  }
  Serial.print("Set ID ");
  if (ID == 255) {
    Serial.print("ALL Connected");
  } else {
    Serial.print(ID, DEC);
  }
  Serial.print(" RS30x servo's BAUDRATE to ");
  Serial.print(tmp);
  Serial.println(".");
  delay(10);                       //待機
}

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

// ■■■■■■■■■■■■■■■ RS30x 返 信 デ ィ レ イ 時 間 の 設 定 ■■■■■■■■■■■■■■■
void RS30x_SetReplayDelay (unsigned char ID, unsigned char dat) {
  unsigned char RS30x_s_data[8];   // 送信データバッファ [9byte]
  unsigned char RS30x_s_cksum = 0; // チェックサム計算用変数

  // 再起動書き込みパケットデータ生成
  RS30x_s_data[0] = 0xFA;          // Header
  RS30x_s_data[1] = 0xAF;          // Header
  RS30x_s_data[2] = ID;            // ID
  RS30x_s_data[3] = 0x00;          // Flags
  RS30x_s_data[4] = 0x07;          // Address
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

  Serial.print("Set ID ");
  if (ID == 255) {
    Serial.print("ALL Connected");
  } else {
    Serial.print(ID, DEC);
  }
  Serial.print(" RS30x servo's REPLY DELAY to ");
  Serial.print(short(dat) * 0.05 + 0.1, 2);
  Serial.println(" ms.");
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
void setup() {
  pinMode(EN_L_PIN, OUTPUT);  // デジタルPin2(EN_L_PIN)を出力に設定
  Serial.begin(60000000);       // Teensy4.0とPCとのシリアル通信速度
  Serial1.begin(115200);    // 現在のシリアルサーボのボーレート（デフォルトは115,200bps）
  //Serial1.begin(230400);    // 変更後のシリアルサーボのボーレート　変換後は新しく設定したボーレートにて開始する
  delay(1000);
  Serial.println("RS303,304 servomotor's setting start.");

  //以下、必要なコマンドをコメントアウトして実行してください
  //設定を書き換える場合には、ROM書き込みと再起動の２行は必ずコメントアウトしてください

  //■ ボーレートの設定 (サーボID(255は全サーボ), ボーレート番号（例は115,200))
  //RS30x_SetSerialSpeed(255, 0x09); //※※※※ボーレート変更にはこの行をコメントアウト※※※※
  // 値の対応表：0x00:9600, 0x01:14,400, 0x02:19,200, 0x03:28.800, 0x04:38,400,
  //           0x05:57,600, 0x06:76,800, 0x07:115,200, 0x08:153,600, 0x09:230,400

  //■ リターンディレイの設定 (サーボID(255は全サーボ), 返信ディレイ 100μs + 数値*0.01（例は0.1ms))
  //RS30x_SetReplayDelay(255, 0x00);//※※※※リターンディレイの変更にはこの行をコメントアウト※※※※

  //■ ROM書き込みと再起動　※下記の２行を設定の際は必ず必要になります
  //RS30x_RomWrite(255);//※※※※何らかの設定を変更する際はこの行をコメントアウト※※※※
  //RS30x_Reboot(255);//※※※※何らかの設定を変更する際はこの行をコメントアウト※※※※

  delay(100);
  //Serial.println("If servos dosn't start moving, Please restart Teensy after change Serial1 baud rate.");
  
  //■ 全サーボトルクオン
  RS30x_Torque(255, 0x01);      // ID = 1(0x01) , RS30x_Torque = ON   (0x01)
  delay(10);
}

// ■■■■■■■■■■■■■■■ M A I N ■■■■■■■■■■■■■■■
void loop() {
  RS30x_Move(255, 100, RS30x_speed); // ID = 1 , GoalPosition = 30.0deg(300) , Time = 1.0sec(100)
  delay(1000);
  RS30x_Move(255, -100, RS30x_speed); // ID = 1 , GoalPosition = -30.0deg(300) , Time = 1.0sec(100)
  delay(1000);
}
