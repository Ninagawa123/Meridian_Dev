// カリカリチューン動く！ Teensy4.0 - (SPI) - ESP32DevKitC 通信
// Teensy用
#include <SPI.h>  // SPIライブラリを導入

//変数の設定
static const int MSG_SIZE = 180;//データのサイズ（何バイト分か）
uint8_t s_message_buf[MSG_SIZE + 4]; //送信データバッファ用の配列
uint8_t r_message_buf[MSG_SIZE + 4]; //受信データバッファ用の配列
int checksum;//チェックサム計算用

long framecount = 0;
long errorcount = -1;
unsigned long time_data = 0;

//SPI通信設定のインスタンスを立てる
SPISettings mySPISettings = SPISettings(6000000, MSBFIRST, SPI_MODE3);

void setup() {
  //SPI通信とシリアル通信の初期設定
  Serial.begin(60000000);
  pinMode (SS, OUTPUT);  // スレーブ機器を起こす
  SPI.begin();   // SPIの初期化
  delay(100); //シリアルの起動を安定させる（要調整）
  Serial.println("SPI Master Start."); //シリアル始動のご挨拶

  //送受信バッファのリセット
  memset(s_message_buf, 0, MSG_SIZE + 4);
  memset(r_message_buf, 0, MSG_SIZE + 4);
}

void loop() {

  //送信データの作成
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) {
    s_message_buf[i] = uint8_t (random(0, 255));
    checksum += int(s_message_buf[i]);
  }
  checksum = (checksum ^ 0xFF) & 0xFF; //合計値を反転し、下位2ビットを取得
  s_message_buf[MSG_SIZE - 1] = uint8_t (checksum); //末尾にチェックサムを追加

  /*
    //送信データの表示
    Serial.print("  [Send] ");
    for (int i = 0; i < MSG_SIZE+4; i++) {
    Serial.print(uint8_t (s_message_buf[i]));
    Serial.print(", ");
    }
    Serial.println();
  */

  //SPI通信の開始
  SPI.beginTransaction(mySPISettings);//通信開始
  digitalWrite(SS, LOW); //スレーブ機器を起こす

  //送信の実施と同時に受信データを受信データ用配列に書き写す
  for ( int i = 0; i < MSG_SIZE + 4; i++) {
    r_message_buf[i] = SPI.transfer(s_message_buf[i]);  //※送信と同時に受信データが返り値になる
    //delayMicroseconds(1);//送受信時間調整用のディレイ
  }


  /*
    //受信データの表示
    Serial.print("  [Rsvd] ");
    for (int i = 0; i < MSG_SIZE+4; i++) {
    Serial.print(uint8_t (r_message_buf[i]));
    Serial.print(", ");
    } Serial.println();
  */
  delayMicroseconds(50);

  //受信データのチェックサム確認
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) {//受信データの末尾-1番までの値を合計
    checksum += int(r_message_buf[i]);
  }
  checksum = (checksum ^ 0xFF) & 0xFF; //合計値を反転し、下位2ビットを取得

  /*
    Serial.print("  CKSUM: "); // チェックサムの正解を表示
    Serial.println(uint8_t (r_message_buf[MSG_SIZE - 1]));
  */

  if (uint8_t (checksum) == uint8_t (r_message_buf[MSG_SIZE - 1])) {//チェックサムの正誤を表示
    //Serial.print("    OK!: "); Serial.println(uint8_t (checksum));
  } else {
    //Serial.print("****NG*: "); Serial.println(uint8_t (checksum));
    errorcount ++;
  }
  Serial.print("ERR ");
  Serial.print(errorcount);
  Serial.print(" / ");
  Serial.print(framecount);
  Serial.println();//シリアルモニタ改行

  time_data = millis();
  Serial.print("SPI ");
  Serial.print(float(framecount) / float(time_data) * 1000);
  Serial.print(" Hz");
  Serial.println();//シリアルモニタ改行

  digitalWrite(SS, HIGH);//スレーブ機器を終了
  SPI.endTransaction();//SPIを解放
  delayMicroseconds(500);
  framecount ++;
}
