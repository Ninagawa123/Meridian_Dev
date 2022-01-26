// カリカリチューン動く！ Teensy4.0 - (SPI) - ESP32DevKitC 通信
// Teensy用
#include <SPI.h>  // SPIライブラリを導入

//変数の設定
static const int MSG_SIZE = 90;//データのサイズ（何バイト分か）
static const int MSG_BUFF = MSG_SIZE * 2; //データのサイズ（何バイト分か）
int checksum;//チェックサム計算用

long framecount = 0;
long errorcount = -1;
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
  for (int i = 0; i < MSG_SIZE - 2; i++) {
    s_spi_meridim.sval[i] = uint8_t (random(-900, 900));
    checksum += int(s_spi_meridim.sval[i]);
  }
  checksum = (checksum ^ 0xFF) & 0xFF; //合計値を反転し、下位2ビットを取得
  s_spi_meridim.sval[MSG_SIZE - 1] = uint8_t (checksum); //末尾にチェックサムを追加

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
  for ( int i = 0; i < MSG_BUFF + 4; i++) {
    r_spi_meridim.bval[i] = SPI.transfer(s_spi_meridim.bval[i]);  //※送信と同時に受信データが返り値になる
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
  for (int i = 0; i < MSG_SIZE - 2; i++) {//受信データの末尾-1番までの値を合計
    checksum += int(r_spi_meridim.sval[i]);
  }
  checksum = (checksum ^ 0xFF) & 0xFF; //合計値を反転し、下位2ビットを取得

  /*
    Serial.print("  CKSUM: "); // チェックサムの正解を表示
    Serial.println(uint8_t (r_message_buf[MSG_SIZE - 1]));
  */

  if (uint8_t (checksum) == uint8_t (r_spi_meridim.sval[MSG_SIZE - 1])) {//チェックサムの正誤を表示
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
  delayMicroseconds(400);
  framecount ++;
}
