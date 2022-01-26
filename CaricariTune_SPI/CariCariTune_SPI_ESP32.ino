// カリカリチューン動く！ Teensy4.0 - (SPI) - ESP32DevKitC 通信
// ESP32用
#include <ESP32DMASPISlave.h>

ESP32DMASPI::Slave slave;

static const int MSG_SIZE = 90;
static const int MSG_BUFF = MSG_SIZE * 2;

uint8_t* s_message_buf;
uint8_t* r_message_buf;
int checksum;
long framecount = 0;
long errorcount = 0;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_spi_meridim; //SPI受信用共用体のインスタンスを宣言
UnionData r_spi_meridim; //SPI受信用共用体のインスタンスを宣言
UnionData s_udp_meridim; //UDP送信用共用体のインスタンスを宣言
UnionData r_udp_meridim; //UDP受信用共用体のインスタンスを宣言

void setup()
{
  Serial.begin(2000000);
  delay(120);//シリアルの開始を待ち安定化させるためのディレイ（要調整）
  Serial.println("SPI Slave Start.");//シリアルモニタの確認用。

  // DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_message_buf = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファを使う
  r_message_buf = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファを使う

  // 送受信バッファをリセット
  memset(s_message_buf, 0, MSG_BUFF + 4);
  memset(r_message_buf, 0, MSG_BUFF + 4);

  //送信データを作成してセット
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 2; i++) //配列の末尾以外をデータを入れる
  {
    short rnd = random(-900, 900);
    s_spi_meridim.sval[i] = rnd;
    checksum += rnd; //チェックサムを加算
  }
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum & 0xFF ^ 0xFF); //データ末尾にチェックサムにする

  for (int i = 0; i < MSG_BUFF - 1 ; i++) { //受信データの転記
    s_message_buf[i] = s_spi_meridim.bval[i] ;
  }

  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF);
  slave.setDMAChannel(2); // 専用メモリの割り当て（1か2のみ)
  slave.setQueueSize(1); // キューサイズ　とりあえず1
  // HSPI(SPI2) のデフォルトピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）
}

void loop()
{

  //このゾーンは高速で回る//

  // キューが送信済みであればセットされた送信データを送信する。
  if (slave.remained() == 0) {
    slave.queue(r_message_buf, s_message_buf, MSG_BUFF + 4);
  }

  // マスターからの送信が終了すると、slave.available()は送信サイズを返し、
  // バッファも自動更新される
  while (slave.available())
  {
    slave.pop();//トランザクションを終了するコマンドらしい

    for (int i = 0; i < MSG_BUFF - 1 ; i++) { //受信データの転記
      r_spi_meridim.bval[i] = int(r_message_buf[i]);
    }

    //受信データのチェックサム確認
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 2 ; i++) { //受信データの末尾-1番までの値を合計
      checksum += int(r_spi_meridim.sval[i]);
    }
    checksum = (checksum ^ 0xff) & 0xff; //合計値を反転し、下位2バイトを取得
    //Serial.print(" cksum: "); Serial.println(uint8_t (r_message_buf[MSG_BUFF - 1]));

    if (checksum == short(r_spi_meridim.sval[MSG_SIZE - 1])) {
      //Serial.print("   OK!: "); Serial.println(uint8_t (checksum));
    } else {
      //Serial.print("**ERR*: "); Serial.println(uint8_t (checksum));
      errorcount ++;
      Serial.print("*err*");
    }
    framecount = framecount + 1;
    Serial.print(errorcount);    Serial.print("/");    Serial.println(framecount);
    //delayMicroseconds(20);//上のシリアル表示しない場合、このディレイを入れる（タイミング調整用）

    //送信データを作成してセット
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 2; i++) //配列の末尾以外をデータを入れる
    {
      short rnd = random(-900, 900);
      s_spi_meridim.sval[i] = rnd;
      checksum += rnd; //チェックサムを加算
    }
    s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum & 0xFF ^ 0xFF); //データ末尾にチェックサムにする

    for (int i = 0; i < MSG_BUFF - 1 ; i++) { //送信データの転記
      s_message_buf[i] = s_spi_meridim.bval[i] ;
    }
  }
}
