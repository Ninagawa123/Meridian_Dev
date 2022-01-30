// カリカリチューン動く！ Teensy4.0-(SPI)-ESP32DevKitC - (Wifi/UDP) - PC/python 通信
// ESP32用

//[E-2] ライブラリ導入 -----------------------------------
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
#include <ESP32DMASPISlave.h>
#include <FastCRC.h>
ESP32DMASPI::Slave slave;

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xxx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
int checksum; //チェックサム計算用
long frame_count = 0;
long error_count_udp = 0;//UDP受信エラーカウント用
long error_count_spi = 0;//SPI受信エラーカウント用
uint8_t* s_spi_meridim_dma;//DMA用
uint8_t* r_spi_meridim_dma;//DMA用

//フラグ関連
bool udp_resv_flag = 1;//UDPスレッドでの受信反応フラグ　1はデータを回し始めるのに必要
//bool udp_revd_flag = 0;//メインスレッドでのUDPの受信処理完了フラグ　
bool udp_send_flag = 1;
bool spi_resv_flag = 0;
bool spi_send_flag = 0;

//マルチスレッドのタスクハンドル格納用
TaskHandle_t thp[4];

//wifi設定
WiFiUDP udp;

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
UnionData r_dummy_meridim; //UDP受信用共用体 テスト用ダミー

void setup()
{
  Serial.begin(2000000);
  delay(130);//シリアルの開始を待ち安定化させるためのディレイ（ほどよい）
  Serial.println("Serial Start...");

  //WiFi 初期化
  WiFi.disconnect(true, true);//WiFi接続をリセット
  Serial.println("Connecting to WiFi to : " + String(AP_SSID));//接続先を表示
  delay(100);
  WiFi.begin(AP_SSID, AP_PASS);//Wifiに接続
  while ( WiFi.status() != WL_CONNECTED) {//https://www.arduino.cc/en/Reference/WiFiStatus 返り値一覧
    delay(50);//接続が完了するまでループで待つ
  }
  Serial.println("WiFi connected.");//WiFi接続完了通知
  Serial.print("ESP32's IP address is  => ");
  Serial.println(WiFi.localIP());//ESP32自身のIPアドレスの表示
  //ESP32自身のBluetoothMacアドレスを表示
  uint8_t bt_mac[6];
  String self_mac_address = "";

  esp_read_mac(bt_mac, ESP_MAC_BT);
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
  Serial.println();

  //UDP通信の開始
  udp.begin(RESV_PORT);
  delay(500);

  // DMAバッファを使う設定　これを使うと一度に送受信できるデータ量を増やせる
  s_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定  ※+4は不具合対策
  r_spi_meridim_dma = slave.allocDMABuffer(MSG_BUFF + 4); //DMAバッファ設定
  //※バッファサイズは4で割り切れる必要があり、なおかつ末尾に4バイト分0が入る不具合があるのでその対策

  // 送受信バッファをリセット
  memset(s_spi_meridim_dma, 0, MSG_BUFF + 4);// ※+4は不具合対策
  memset(r_spi_meridim_dma, 0, MSG_BUFF + 4);



  //初回の送信データを作成してセット
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) //配列の末尾以外にデータを入れる
  {
    short rnd = random(-30000, 30000);//ここではランダムな配列を使う
    s_spi_meridim.sval[i] = rnd;//配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
  }
  checksum = short(~checksum);//チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum); //データ末尾にチェックサムを入れる
  for (int i = 0; i < MSG_BUFF + 4; i++) { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
  }

  //ここでスレッドを立てる宣言（無線はすべてCORE0で動く。割り込みも含め。メインループはCORE1）
  xTaskCreatePinnedToCore(Core0_UDP_r, "Core0_UDP_r", 4096, NULL, 4, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0_UDP_s, "Core0_UDP_s", 4096, NULL, 3, &thp[1], 0);

  slave.setDataMode(SPI_MODE3);
  slave.setMaxTransferSize(MSG_BUFF + 4);
  slave.setDMAChannel(2); // 専用メモリの割り当て（1か2のみ)
  slave.setQueueSize(1); // キューサイズ　とりあえず1
  // HSPI(SPI2) のデフォルトピン番号は CS: 15, CLK: 14, MOSI: 13, MISO: 12
  slave.begin(); // 引数を指定しなければデフォルトのSPI（SPI2,HSPIを利用）
}


//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// ■ 受信用の関数 パケットが来ていれば、RECVDATANUM 分だけr_ufdata配列に保存 -----
void receiveUDP() {
  int packetSize = udp.parsePacket();
  byte tmpbuf[MSG_BUFF];

  //データの受信
  if (packetSize == MSG_BUFF) {
    udp.read(tmpbuf, MSG_BUFF);
    for (int i = 0; i < MSG_BUFF; i++)
    {
      r_udp_meridim.bval[i] = tmpbuf[i];
    }
    udp_resv_flag = 1;
  }
}

// ■ 送信用の関数 ----------------------------------------------------------
void sendUDP() {
  udp.beginPacket(SEND_IP, SEND_PORT);//UDPパケットの開始
  for (int i = 0; i < MSG_BUFF; i++) {
    udp.write(s_udp_meridim.bval[i]);//１バイトずつ送信
  }
  udp.endPacket();//UDPパケットの終了
}


//-------------------------------------------------------------------------
//---- U D P 用 ス レ ッ ド ------------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_r(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    receiveUDP();
    delay(1);//1/1000秒待つ
  }
}

void Core0_UDP_s(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {
    if (udp_send_flag == 1) {
      sendUDP();
      udp_send_flag = 0;
    }
    delay(1);//1/1000秒待つ
  }

}

//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //---- < 1 > U D P 受 信 --------------------------------------------------
  //[1-1] UDP受信の実行 もしデータパケットが来ていれば受信する
  //receiveUDP();
  int udp_resv_wait = 0;
  while (udp_resv_flag != 1) {
    //delay(1);
    delayMicroseconds(1);
    udp_resv_wait ++;
    if (udp_resv_wait > 2000000) { //UPD受信待ちは2秒でタイムアウトし次へ
      udp_resv_flag = 1;//タイムアウトでUPD受信完了フラグを上げる（あとでエラーコードも足す）
    }
  }

  //UDPを受信したら以下を実行
  //[1-2] UDP受信配列からSPI送信配列にデータを転写
if (udp_resv_flag == 1) {
  for (int i = 0; i < MSG_BUFF; i++) {
    r_dummy_meridim.bval[i] = r_udp_meridim.bval[i];
  }

  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += int(r_dummy_meridim.sval[i]);
  }
  checksum = short(~checksum);//チェックサムを計算

  if (checksum == r_dummy_meridim.sval[MSG_SIZE - 1])
  {
  } else {
    error_count_udp ++;
  }
}

  //---- < 2 > S P I 送 信 信 号 作 成  -----------------------------------------------




  //---- < 3 > S P I 送 受 信 -----------------------------------------------
  while (slave.available())//完了した（受信した）トランザクション結果のサイズ size of completed (received) transaction results
  {
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
    Serial.print("U:"); Serial.println(error_count_udp);//    Serial.print("/");    Serial.println(frame_count);
    Serial.print("S:"); Serial.println(error_count_spi);//    Serial.print("/");    Serial.println(frame_count);

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
    slave.pop();//DMAのデータ配列の先頭を削除
  }

  // キューが送信済みであればセットされた送信データを送信する。
  if (slave.remained() == 0) {//キューに入れられた（完了していない）トランザクションのサイズが0なら実行　size of queued (not completed) transactions
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    spi_resv_flag = 1;//SPI受信完了フラグを上げる
  }


  //---- < 4 > U D P 送 信 信 号 作 成 --------------------------------------------------
  if (spi_resv_flag == 1) {
    for (int i = 0; i < MSG_SIZE - 2; i++) //配列の末尾以外をデータを入れる
    {
      short rnd = random(-30000, 30000);
      s_udp_meridim.sval[i] = rnd;
    }
    // チェックサムを計算して格納
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i ++) {
      checksum += s_udp_meridim.sval[i];
    }
    checksum = short(~checksum);//チェックサムを計算
    s_udp_meridim.sval[MSG_SIZE - 1] = short(checksum);

    //---- < 5 > U D P 送 信 ----------------------------------------------
    //[4-1] UDP送信を実行

    udp_send_flag = 1;//UDP送信準備完了フラグを上げる
    spi_resv_flag = 0;//SPI受信完了フラグを下げる
  }

  delayMicroseconds(1);
}
