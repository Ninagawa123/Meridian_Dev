// カリカリチューン動く！ Teensy4.0 - (SPI) - ESP32DevKitC 通信
// ESP32用　2022.01.29
// short型に対応
// チェックサムも上手くいっているこの式にすべてあわせること

#include <ESP32DMASPISlave.h>
#include <FastCRC.h>
ESP32DMASPI::Slave slave;

static const int MSG_SIZE = 90;
static const int MSG_BUFF = MSG_SIZE * 2;

uint8_t* s_spi_meridim_dma;
uint8_t* r_spi_meridim_dma;
int checksum;
long frame_count = 0;
long error_count_spi = 0;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE + 2];
  uint8_t bval[MSG_BUFF + 4];
} UnionData;
UnionData s_spi_meridim; //SPI受信用共用体
UnionData r_spi_meridim; //SPI受信用共用体
UnionData s_udp_meridim; //UDP送信用共用体
UnionData r_udp_meridim; //UDP受信用共用体

void setup()
{
  Serial.begin(2000000);
  delay(120);//シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println("SPI Slave Start.");//シリアルモニタの確認用

  // DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定 +4は取りこぼし対策
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定

  // 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4);// +4は取りこぼし対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);

  //初回の送信データを作成してセット
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    short rnd = random(-30000, 30000);//ここではランダムな配列を使う
    s_spi_meridim.sval[i] = rnd;//配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
  }
  s_spi_meridim.sval[MSG_SIZE - 1] = short(~checksum); //データ末尾にチェックサムを入れる

  for (int i = 0; i < MSG_BUFF + 4; i++) { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
  }

  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て（1か2のみ)
  slave.setQueueSize(1); // キューサイズ　とりあえず1
  // HSPI(SPI2) のデフォルトピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）
}

void loop()
{

  //この行にコードを記入すると高速で回るので注意//

  // キューが送信済みであればセットされた送信データを送信する。
  if (slave.remained() == 0) {
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
  }

  // マスターからの送信が終了すると、slave.available()は送信サイズを返し、
  // バッファも自動更新される
  while (slave.available())
  {
    slave.pop();//トランザクションを終了するコマンド？

    for (int i = 0; i < MSG_BUFF ; i++) { //受信データの転記
      r_spi_meridim.bval[i] = int(r_spi_meridim_dma[i]);
    }

    //受信データのチェックサム確認
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1 ; i++) { //受信データの末尾-1番までの値を合計
      checksum += int(r_spi_meridim.sval[i]);
    }
    checksum = short(~checksum);//チェックサムを計算
    if (checksum == r_spi_meridim.sval[MSG_SIZE - 1]) {
    } else {
      error_count_spi ++;
    }
    frame_count = frame_count + 1;
    Serial.print(error_count_spi);    Serial.print("/");    Serial.println(frame_count);

    //送信データを作成してセット
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
    {
      short rnd = random(-30000, 30000);//ここではランダムな配列を使う
      s_spi_meridim.sval[i] = rnd;//配列にランダムなデータを入れる
      checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
    }
    checksum = short(~checksum);//チェックサムを確定
    s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum); //データ末尾にチェックサムを入れる
    for (int i = 0; i < MSG_BUFF; i++) { //受信データのDMAから配列への転記
      s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
    }
  }
}
