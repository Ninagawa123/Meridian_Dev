// カリカリチューン動く！ ESP32DevKitC - (Wifi/UDP) - PC/python  通信
// ESP32用

//[E-2] ライブラリ導入 -----------------------------------
#include <WiFi.h>//UDPの設定
#include <WiFiUdp.h>//UDPの設定
#include <ESP32DMASPISlave.h>
#include <FastCRC.h>
ESP32DMASPI::Slave slave;
#include <PS4Controller.h>//PS4コントローラー
#include <ESP32Wiimote.h>
ESP32Wiimote wiimote;

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define AP_SSID "xxxxxxxx" //アクセスポイントのAP_SSID
#define AP_PASS "xxxxxxxx" //アクセスポイントのパスワード
#define SEND_IP "192.168.1.xx" //送り先のPCのIPアドレス（PCのIPアドレスを調べておく）
#define SEND_PORT 22222 //送り先のポート番号
#define RESV_PORT 22224 //このESP32のポート番号
#define JOYPAD_MOUNT 2 ////ジョイパッドの搭載 0:なし、1:Wii_yoko, 2:Wii+Nun, 3:PS3, 4:PS4, 5:WiiPRO, 6:Xbox
#define JOYPAD_POLLING 10 ////ジョイパッドの問い合わせフレーム間隔(ms)

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

//コントローラー用変数
unsigned short pad_btn = 0;
short pad_stick_R = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_stick_V = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;

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

  //コントローラの接続開始
  //PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4) {
    PS4.begin("e8:68:e7:30:b4:52");//ESP32のMACが入ります.PS4にも設定します。
    Serial.println("PS4 controller connecting...");
  }
  //Wiiコントローラの接続開始
  if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
    wiimote.init();

    wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
    Serial.println("Wiimote connecting...");
  }
  delay(1000);

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
    s_spi_meridim.sval[i] = 0;//配列にランダムなデータを入れる
    checksum += int(s_spi_meridim.sval[i]); //チェックサムを加算
  }
  checksum = short(~checksum);//チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum); //データ末尾にチェックサムを入れる
  for (int i = 0; i < MSG_BUFF + 4; i++) { //送信データをDMAバッファに転記
    s_spi_meridim_dma[i] = s_spi_meridim.bval[i] ;
  }

  //ここでスレッドを立てる宣言（無線はすべてCORE0で動く。割り込みも含め。メインループはCORE1）
  xTaskCreatePinnedToCore(Core0_UDP_r, "Core0_UDP_r", 8192, NULL, 10, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0_UDP_s, "Core0_UDP_s", 4096, NULL, 7, &thp[1], 0);
  xTaskCreatePinnedToCore(Core0_BT_r, "Core0_BT_r", 8192, NULL, 5, &thp[2], 0);

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

// ■ 受信用の関数 パケットが来ていれば、MSG_BUFF 分だけr_udp_meridim.bval配列に保存 -----
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
//---- U D P 受 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_r(void *args) {//サブCPU(Core0)で実行するプログラム
  delay(1000);

  while (1) {//ここで無限ループを作っておく
    //receiveUDP();
    delay(1);//1/1000秒待つ
  }
}

//-------------------------------------------------------------------------
//---- U D P 送 信 用 ス レ ッ ド -------------------------------------------
//-------------------------------------------------------------------------
void Core0_UDP_s(void *args) {//サブCPU(Core0)で実行するプログラム
  delay(1000);

  while (1) {
    if (udp_send_flag == 1) {
      sendUDP();
      udp_send_flag = 0;
    }
    delay(6);//1/1000秒待つ
  }

}

// ■ PS4コントローラ受信用関数 ------------------------------------------------

void PS4pad_receive() {
  // Below has all accessible outputs from the controller
  if (PS4.isConnected()) {
    pad_btn = 0;
    if (PS4.Right())    pad_btn |= (B00000000 * 256) + B00100000;
    if (PS4.Down())     pad_btn |= (B00000000 * 256) + B01000000;
    if (PS4.Up())       pad_btn |= (B00000000 * 256) + B00010000;
    if (PS4.Left())     pad_btn |= (B00000000 * 256) + B10000000;

    if (PS4.Square())   pad_btn |= (B10000000 * 256) + B00000000;
    if (PS4.Cross())    pad_btn |= (B01000000 * 256) + B00000000;
    if (PS4.Circle())   pad_btn |= (B00100000 * 256) + B00000000;
    if (PS4.Triangle()) pad_btn |= (B00010000 * 256) + B00000000;

    if (PS4.UpRight())  pad_btn |= (B00000000 * 256) + B00110000;
    if (PS4.DownRight())pad_btn |= (B00000000 * 256) + B01100000;
    if (PS4.UpLeft())   pad_btn |= (B00000000 * 256) + B10010000;
    if (PS4.DownLeft()) pad_btn |= (B00000000 * 256) + B11000000;

    if (PS4.L1())       pad_btn |= (B00000100 * 256) + B00000000;
    if (PS4.R1())       pad_btn |= (B00001000 * 256) + B00000000;

    if (PS4.Share())    pad_btn |= (B00000000 * 256) + B00000001;
    if (PS4.Options())  pad_btn |= (B00000000 * 256) + B00001000;
    if (PS4.L3())       pad_btn |= (B00000000 * 256) + B00000100;
    if (PS4.R3())       pad_btn |= (B00000000 * 256) + B00000010;

    if (PS4.PSButton()) pad_btn |= (B00000000 * 256) + B01010000;//same as up & down
    if (PS4.Touchpad()) pad_btn |= (B00000000 * 256) + B00101000;//same as left & right

    if (PS4.L2()) {
      pad_btn |= (0x00000001 * 256) + B00000000;
      pad_L2_val = constrain(PS4.L2Value(), 0, 255);
    }
    if (PS4.R2()) {
      pad_btn |= 512;//(0x00000010 * 256) + B00000000;
      pad_R2_val = constrain(PS4.R2Value(), 0, 255);
    }

    if (PS4.LStickX()) {
      pad_stick_L_x = constrain(PS4.LStickX(), -127, 127);
    }
    if (PS4.LStickY()) {
      pad_stick_L_y = constrain(PS4.LStickY(), -127, 127);
    }
    if (PS4.RStickX()) {
      pad_stick_R_x = constrain(PS4.RStickX(), -127, 127);
    }
    if (PS4.RStickY()) {
      pad_stick_R_y = constrain(PS4.RStickY(), -127, 127);
    }

    pad_stick_L = pad_stick_L_x * 256 + pad_stick_L_y;
    pad_stick_R = pad_stick_R_x * 256 + pad_stick_R_y;
    pad_stick_V = pad_L2_val * 256 + pad_R2_val;
  }
}

// ■ Wiiコントローラ(横持ち・横持ち+ヌンチャク)受信用関数 -----------------------------

void Wiipad_receive_h() {
  wiimote.task();
  if (wiimote.available() > 0) {
    uint16_t button = wiimote.getButtonState();
    NunchukState nunchuk = wiimote.getNunchukState();
    pad_btn = 0;

    if (button & 0x0400) pad_btn |= (B00000000 * 256) + B00100000; //right
    if (button & 0x0100) pad_btn |= (B00000000 * 256) + B01000000; //down
    if (button & 0x0200) pad_btn |= (B00000000 * 256) + B00010000; //up
    if (button & 0x0800) pad_btn |= (B00000000 * 256) + B10000000; //left

    if (button & 0x0008) pad_btn |= (B10000000 * 256) + B00000000; //A
    if (button & 0x0002) pad_btn |= (B01000000 * 256) + B00000000; //1
    if (button & 0x0001) pad_btn |= (B00100000 * 256) + B00000000; //2
    if (button & 0x0004) pad_btn |= (B00010000 * 256) + B00000000; //B

    if (button & 0x0010) pad_btn |= (B00000000 * 256) + B00000001; //+
    if (button & 0x1000) pad_btn |= (B00000000 * 256) + B00001000; //-

    if (button & 0x0080) pad_btn |= (B00000000 * 256) + B01010000; //same as up & down//home

    if (JOYPAD_MOUNT == 2) {
      NunchukState nunchuk = wiimote.getNunchukState();
      pad_stick_L = (nunchuk.xStick * 256 + nunchuk.yStick);
      if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
    delay(6);//ここの数値でCPU負荷を軽減できるかも
  }
}

//-------------------------------------------------------------------------
//---- Bluetooth 用 ス レ ッ ド --------------------------------------------
//-------------------------------------------------------------------------

void Core0_BT_r(void *args) {//サブCPU(Core0)で実行するプログラム
  while (1) {//ここで無限ループを作っておく
    Wiipad_receive_h();
    delay(1);//JOYPAD_POLLING ms秒待つ
  }
}

//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //---- < 1 > U D P 受 信 --------------------------------------------------
  //[1-1] UDP受信の実行 もしデータパケットが来ていれば受信する
  receiveUDP();
  /*
  int udp_resv_wait = 0;
  while (udp_resv_flag != 1) {
    //delay(1);
    delay(1);
    udp_resv_wait ++;
    if (udp_resv_wait > 3000) { //UPD受信待ちは3秒でタイムアウトし次へ
      udp_resv_flag = 1;//タイムアウトでUPD受信完了フラグを上げる（あとでエラーコードも足す）
    }
  }
  */

  //ここで「r_udp_meridim」に中身が入った状態

  //UDPを受信したら以下を実行
  //[1-2] UDP受信配列からSPI送信配列にデータを転写
  if (udp_resv_flag == 1) {

    //ここで「r_udp_meridim」のチェックサムを実施
    checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i ++) {
      checksum += int(r_udp_meridim.sval[i]);
    }
    checksum = short(~checksum);//チェックサムを計算

    if (r_udp_meridim.sval[MSG_SIZE - 1] == short(checksum))
    {
      //SPI送信データを上書き更新する
      memcpy(s_spi_meridim.bval, r_udp_meridim.bval, MSG_BUFF + 4);
      //ここでこのパートのエラーフラグも消す
      //※ここのデータはあとでやる。エクセルの表を見てフラグを立てる作業
      Serial.println("UDPrvOK!");
    } else {
      //前回のSPI送信データにエラーフラグだけ上乗せする
      //※ここのデータはあとでやる。エクセルの表を見てフラグを立てる作業
      error_count_udp ++;
      Serial.print("U:"); Serial.println(error_count_udp);//    Serial.print("/");    Serial.println(frame_count);
    }
    udp_resv_flag =0;
  }

  //ここで「s_spi_meridim」にはチェックサム済みの「r_udp_meridim」が転記され、ESP32UDP受信エラーフラグも入った状態

  //---- < 2 > S P I 送 信 信 号 作 成  -----------------------------------------------

  //リモコンデータの書き込み ★場所はここで良いのか後で確認？たぶんよい？
  s_spi_meridim.sval[80] = pad_btn;
  if (s_spi_meridim.sval[80] != 0){
    Serial.println(s_spi_meridim.sval[80],BIN);
  }
  s_spi_meridim.sval[81] = pad_stick_L;
  s_spi_meridim.sval[82] = pad_stick_R;
  s_spi_meridim.sval[83] = pad_stick_V;

  //ここで「s_spi_meridim」にはESP32で受けたリモコンデータも入った状態

  //ここで「s_spi_meridim」のチェックサムを作成
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += int(s_spi_meridim.sval[i]);
  }
  checksum = short(~checksum);//チェックサムを計算
  s_spi_meridim.sval[MSG_SIZE - 1] = short(checksum);

  //ここで「s_spi_meridim」はチェックサムも入り完成している

  //---- < 3 > S P I 送 受 信 -----------------------------------------------
  while (slave.available())//完了した（受信した）トランザクション結果のサイズ size of completed (received) transaction results
  {
    //SPI送信データが完成し、DMA転記も完了
    memcpy(s_spi_meridim_dma, s_spi_meridim.bval, MSG_BUFF + 4);
    memcpy(r_spi_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4);
    //ここで「r_spi_meridim.bval」に中身が入った状態
    //SPI受信の中身はチェックせずそのままUDPでPCにパススルーする

    frame_count = frame_count + 1;

    slave.pop();//DMAのデータ配列の先頭を削除
  }

  //ここでSPIの送受信を行う
  // キューが送信済みであればセットされた送信データを送信する。
  if (slave.remained() == 0) {//キューに入れられた（完了していない）トランザクションのサイズが0なら実行　size of queued (not completed) transactions
    slave.queue(r_spi_meridim_dma, s_spi_meridim_dma, MSG_BUFF + 4);
    spi_resv_flag = 1;//SPI受信完了フラグを上げる
  }

  //---- < 4 > U D P 送 信 信 号 作 成 ----------------------------------------
  if (spi_resv_flag == 1) {

    memcpy(s_udp_meridim.bval, r_spi_meridim_dma, MSG_BUFF + 4);

    //ここで「s_udp_meridim.bval」に中身が入った状態

    //---- < 5 > U D P 送 信 ----------------------------------------------
    //[4-1] UDP送信を実行（フラグを立ててスレッドに知らせる）
    udp_send_flag = 1;//UDP送信準備完了フラグを上げる
    spi_resv_flag = 0;//SPI受信完了フラグを下げる
  }
  delayMicroseconds(1);
}
