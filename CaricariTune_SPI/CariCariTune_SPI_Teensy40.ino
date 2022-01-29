// カリカリチューン動く！ Teensy4.0 - (SPI) - ESP32DevKitC 通信
// Teensy用 2022.01.29
// short型に対応
// チェックサムも上手くいっているこの式にすべてあわせること

#include <SPI.h>  // SPIライブラリを導入

//変数の設定
static const int MSG_SIZE = 90;//データのサイズ（何バイト分か）
static const int MSG_BUFF = MSG_SIZE * 2; //データのサイズ（何バイト分か）
int checksum;//チェックサム計算用

long frame_count = 0;
long error_count_spi = 0;
unsigned long time_data = 0;

//共用体の宣言 : Meridim配列格納用、SPI送受信バッファ配列格納用
typedef union //共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
  short sval[MSG_SIZE + 2]; // short型で100個の配列データを持つ
  uint8_t bval[MSG_BUFF + 4]; //1バイト単位で200個の配列データを持つ
} UnionData;

UnionData s_spi_meridim; //Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_spi_meridim; //Meridim配列データ(short型、センサや角度は100倍値)

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
  memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);
  memset(r_spi_meridim.bval, 0, MSG_BUFF + 4);
}

void loop() {

  //送信データの作成
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    short rnd = random(-30000, 30000);//ここではランダムな配列を使う
    s_spi_meridim.sval[i] = rnd;//配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]);
  }
  checksum = short(~checksum);//チェックサムを確定

  s_spi_meridim.sval[MSG_SIZE - 1] = short (checksum); //末尾にチェックサムを追加

  //SPI通信の開始
  SPI.beginTransaction(mySPISettings);//通信開始
  digitalWrite(SS, LOW); //スレーブ機器を起こす

  //送信の実施と同時に受信データを受信データ用配列に書き写す
  for ( int i = 0; i < MSG_BUFF + 4; i++) {
    r_spi_meridim.bval[i] = SPI.transfer(s_spi_meridim.bval[i]);  //※送信と同時に受信データが返り値になる
    //delayMicroseconds(1);//送受信時間調整用のディレイ
  }
  delayMicroseconds(50);

  //受信データのチェックサム確認
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) {//受信データの末尾-1番までの値を合計
    checksum += int(r_spi_meridim.sval[i]);
  }
  checksum = short(~checksum);//チェックサムを計算
  if (checksum == r_spi_meridim.sval[MSG_SIZE - 1]) {//チェックサムの正誤を表示
  } else {
    error_count_spi ++;
  }
  
  Serial.print("Err ");
  Serial.print(error_count_spi);
  Serial.print(" / ");
  Serial.print(frame_count);
  Serial.println();//シリアルモニタ改行

  time_data = millis();
  Serial.print("Spd ");
  Serial.print(float(frame_count) / float(time_data) * 1000);
  Serial.print(" Hz");
  Serial.println();//シリアルモニタ改行

  digitalWrite(SS, HIGH);//スレーブ機器を終了
  SPI.endTransaction();//SPIを解放
  delayMicroseconds(400);
  frame_count ++;
}
