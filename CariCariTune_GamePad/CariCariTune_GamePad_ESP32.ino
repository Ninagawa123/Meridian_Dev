// カリカリチューン動く！ ESP32DevKitC - (Bluetooth) - PS4/Wiiリモコン 通信
// ESP32用
//
//【注意点】
//★PS4コントローラは20220130現在使用不能（今月からペアリング確立しなくなった）
//★ヌンチャクのデータも一応表示するが、レバーの取得が適切ではない
//★よってWiiリモコン横持ちのみ適切に反応

//[E-2] ライブラリ導入 -----------------------------------
#include <PS4Controller.h>//PS4コントローラー
#include <ESP32Wiimote.h>
ESP32Wiimote wiimote;

//[E-3] 各種設定 #DEFINE ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define JOYPAD_MOUNT 2 ////ジョイパッドの搭載 0:なし、1:Wii_yoko, 2:Wii+Nun, 3:PS3, 4:PS4, 5:WiiPRO, 6:Xbox
#define JOYPAD_POLLING 10 ////ジョイパッドの問い合わせフレーム間隔

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2;
uint8_t* s_message_buf;
uint8_t* r_message_buf;
int checksum; //チェックサム計算用
long frame_count = 0;
long error_count = 0;
unsigned long time_data = 0;

//共用体の設定。共用体はたとえばデータをショートで格納し、バイト型として取り出せる
typedef union
{
  short sval[MSG_SIZE];
  uint8_t bval[MSG_BUFF];
} UnionData;
UnionData s_upd_message_buf; //送信用共用体のインスタンスを宣言
UnionData r_upd_message_buf; //受信用共用体のインスタンスを宣言
UnionData r_spi_message_buf;

//コントローラー用変数
unsigned short pad_btn = 0;
short pad_stick_R_x = 0;
short pad_stick_R_y = 0;
short pad_stick_L_x = 0;
short pad_stick_L_y = 0;
short pad_R2_val = 0;
short pad_L2_val = 0;
long pad_btn_disp = 65536;
int joypad_count = 0; ////ジョイパッドの問い合わせフレームカウント用

void setup()
{
  Serial.begin(2000000);
  delay(120);//シリアル準備待ち用ディレイ
  Serial.println("Serial Start...");

  //ESP32自身のBluetoothMacアドレスを表示
  uint8_t bt_mac[6];
  String self_mac_address = "";

  esp_read_mac(bt_mac, ESP_MAC_BT);
  self_mac_address = String(bt_mac[0], HEX) + ":" + String(bt_mac[1], HEX) + ":" + String(bt_mac[2], HEX) + ":" + String(bt_mac[3], HEX) + ":" + String(bt_mac[4], HEX) + ":" + String(bt_mac[5], HEX);
  Serial.print("ESP32's Bluetooth Mac Address is => " + self_mac_address);
  Serial.println();

  //コントローラの接続開始
  //PS4コントローラの接続開始
  if (JOYPAD_MOUNT == 4) {
    PS4.begin("xx:xx:xx:xx:xx:xx");//ESP32のMACが入ります.PS4にも設定します。
  }
  //Wiiコントローラの接続開始
  if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
    wiimote.init();
    wiimote.addFilter(ACTION_IGNORE, FILTER_NUNCHUK_ACCEL);
  }
}


//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// ■ PS4コントローラ受信用関数 ----------------------------------------------------------

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

    //if (PS4.Charging()) Serial.println("The controller is charging");
    //if (PS4.Audio()) Serial.println("The controller has headphones attached");
    //if (PS4.Mic()) Serial.println("The controller has a mic attached");
    //Serial.printf("Battery Level : %d\n", PS4.Battery());

    s_upd_message_buf.sval[80] = pad_btn;
    s_upd_message_buf.sval[81] = pad_stick_L_x * 256 + pad_stick_L_y;
    s_upd_message_buf.sval[82] = pad_stick_R_x * 256 + pad_stick_R_y;
    s_upd_message_buf.sval[83] = pad_L2_val * 256 + pad_R2_val;

    /*
      pad_btn_disp = pad_btn + 65536;
      Serial.print(pad_btn_disp, BIN);
      Serial.print(", ");
      Serial.print(pad_stick_L_x);
      Serial.print(", ");
      Serial.print(pad_stick_L_y);
      Serial.print(", ");
      Serial.print(pad_stick_R_x);
      Serial.print(", ");
      Serial.print(pad_stick_R_y);
      Serial.print(", ");
      Serial.print(pad_L2_val);
      Serial.print(", ");
      Serial.println(pad_R2_val);
      delay(10);
    */
  }
}

// ■ Wiiコントローラ(横持ち)受信用関数 ----------------------------------------------------------

void Wiipad_receive_h() {

  wiimote.task();
  if (wiimote.available() > 0) {
    uint16_t button = wiimote.getButtonState();
    NunchukState nunchuk = wiimote.getNunchukState();

    //ボタンを押すとサーボ位置のL14の値がプラマイ反転する不具合あり
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

    //ヌンチャク
    s_upd_message_buf.sval[81] = 0;//pad_stick_L
    s_upd_message_buf.sval[82] = 0;//pad_stick_R
    s_upd_message_buf.sval[83] = 0;//pad_L2_R2_val

    if (JOYPAD_MOUNT == 2) {
      NunchukState nunchuk = wiimote.getNunchukState();
      s_upd_message_buf.sval[81] = (nunchuk.xStick * 256 + nunchuk.yStick);
      if (nunchuk.cBtn == 1) pad_btn |= (B00000100 * 256) + B00000000;
      if (nunchuk.zBtn == 1) pad_btn |= (0x00000001 * 256) + B00000000;
    }
  }
  s_upd_message_buf.sval[80] = pad_btn;
}

//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop()
{
  //---- < 1 > U D P 受 信 --------------------------------------------------

  //---- < 2 > S P I 送 信 信 号 作 成  -----------------------------------------------

  //---- < 3 > 受信データの加工 -----------------------------------------------
  // SPI受信配列からUDP送信配列にデータを転写
  for (int i = 0; i < MSG_BUFF; i++) {
    s_upd_message_buf.bval[i] = r_spi_message_buf.bval[i];
  }

  // 受信チェックサムの計算
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) {
    checksum += s_upd_message_buf.sval[i];
  }
  checksum = ~checksum & 0xffff;


  //s_upd_message_buf.sval[88] = s_upd_message_buf.sval[88] & (B10111111 * 256) + B11111111;//エラー配列14番(ESP32UDP受信エラー)をオフに
  // 送信データを加工(リモコン受信データの差し込みなど)

  joypad_count ++ ;
  if (joypad_count >= JOYPAD_POLLING)
  {
    //コントローラの受信
    if (JOYPAD_MOUNT == 4) {
      PS4pad_receive();
    }
    if ((JOYPAD_MOUNT == 1) or (JOYPAD_MOUNT == 2)) {
      Wiipad_receive_h();
    }
  }

  //---- < 3 > S P I 送 受 信 -----------------------------------------------

  //---- < 4 > U D P 送 信 信 号 作 成 --------------------------------------------------

  //---- < 5 > U D P 送 信 ----------------------------------------------
  Serial.println(s_upd_message_buf.sval[80], BIN);

  //★ヌンチャクのデータも一応表示するが、レバーの取得が適切ではない
  int nun_x_center = 133;
  int nun_y_center = 136;
  Serial.print((s_upd_message_buf.sval[81] >> 8 & 0x00ff));// - nun_x_center);
  Serial.print(",");
  Serial.println((s_upd_message_buf.sval[81] & 0x00ff));// - nun_y_center);

  delayMicroseconds(1);

  time_data = millis();
  Serial.print("BT ");
  Serial.print(float(frame_count) / float(time_data) * 1000);
  Serial.print(" Hz");
  Serial.println();//シリアルモニタ改行
  delayMicroseconds(500);
  frame_count ++;
  delay(1);//このディレイは外すことができる
}
