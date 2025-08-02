#include <mcp_can.h>
#include <SPI.h>

// PI定数の定義（Arduino環境で未定義の場合）
#ifndef PI
#define PI 3.14159265359f
#endif

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);
// CyberGear モータの設定
const uint8_t MOTOR_ID = 0x7F;  // モータのID（1-127）
const uint8_t MASTER_ID = 0x00; // マスターID
// CyberGearプロトコルタイプ（マニュアル準拠）
const uint8_t COMM_GET_DEVICE_ID = 0;
const uint8_t COMM_MOTOR_CONTROL = 1;
const uint8_t COMM_MOTOR_FEEDBACK = 2;
const uint8_t COMM_MOTOR_ENABLE = 3;
const uint8_t COMM_MOTOR_DISABLE = 4;
const uint8_t COMM_SET_MECH_ZERO = 6;
const uint8_t COMM_PARAM_WRITE = 18;

// パラメータインデックス（マニュアル準拠）
const uint16_t PARAM_RUN_MODE = 0x7005;
const uint16_t PARAM_IQ_REF = 0x7006;
const uint16_t PARAM_SPD_REF = 0x700A;
const uint16_t PARAM_LIMIT_TORQUE = 0x700B;
const uint16_t PARAM_LOC_REF = 0x7016;
const uint16_t PARAM_LIMIT_SPD = 0x7017;
const uint16_t PARAM_LIMIT_CUR = 0x7018;
const uint16_t PARAM_MECH_POS = 0x7019;
const uint16_t PARAM_IQF = 0x701A;
const uint16_t PARAM_MECH_VEL = 0x701B;
// 制御モード
const uint8_t MODE_MOTION = 0x00;
const uint8_t MODE_POSITION = 0x01;
const uint8_t MODE_SPEED = 0x02;
const uint8_t MODE_CURRENT = 0x03;
// パラメータの範囲（マニュアル準拠）
const float P_MIN = -12.5f;
const float P_MAX = 12.5f;
const float V_MIN = -30.0;
const float V_MAX = 30.0;
const float T_MIN = -12.0;
const float T_MAX = 12.0;
const float KP_MIN = 0.0;
const float KP_MAX = 500.0;
const float KD_MIN = 0.0;
const float KD_MAX = 5.0;

// CAN ID生成関数（マニュアル準拠）
uint32_t make_can_id(uint8_t comm_type, uint16_t data, uint8_t motor_id) {
  return ((uint32_t)comm_type << 24) | ((uint32_t)data << 8) | motor_id;
}
// 現在の状態
float current_position = 0.0;
float current_velocity = 0.0;
float current_torque = 0.0;
unsigned long last_command_time = 0;
unsigned long last_status_request = 0;
void setup() {
  Serial.begin(115200);
  
  // CAN初期化
  if(CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN: Init OK!");
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("CAN: Init Fail!");
    while(1);
  }
  
  delay(1000);
  
  // モータを有効化
  enableMotor();
  delay(100);
  
  // メカニカルゼロ設定
  setMechanicalZero();
  delay(100);
  
  Serial.println("CyberGear Motor Ready!");
  Serial.println("Commands:");
  Serial.println("e - Enable motor");
  Serial.println("d - Disable motor");
  Serial.println("z - Set mechanical zero");
  Serial.println("p<value> - Set position (e.g., p1.57)");
  Serial.println("v<value> - Set velocity (e.g., v5.0)");
  Serial.println("t<value> - Set torque (e.g., t2.0)");
  Serial.println("s - Get status");
}
void loop() {
  // シリアルコマンド処理
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // CANメッセージ受信処理
  if(CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    byte len;
    byte rxBuf[8];
    
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    processCanMessage(rxId, len, rxBuf);
  }
  
  // 定期的にステータス要求（1秒間隔）
  if(millis() - last_status_request > 1000) {
    requestStatus();
    last_status_request = millis();
  }
}
void processCommand(String cmd) {
  if(cmd == "e") {
    enableMotor();
  } else if(cmd == "d") {
    disableMotor();
  } else if(cmd == "z") {
    setMechanicalZero();
  } else if(cmd == "s") {
    requestStatus();
  } else if(cmd.startsWith("p")) {
    float pos = cmd.substring(1).toFloat();
    // 位置モードに切り替えて位置制御
    setRunMode(MODE_POSITION);
    delay(10);
    writeParameter(PARAM_LIMIT_SPD, 5.0); // 速度制限
    delay(10);
    writeParameter(PARAM_LOC_REF, pos); // 位置コマンド
    Serial.print("Position set to: ");
    Serial.print(pos, 3);
    Serial.println(" rad");
  } else if(cmd.startsWith("v")) {
    float vel = cmd.substring(1).toFloat();
    setVelocity(vel, 2.0); // velocity, torque_limit
  } else if(cmd.startsWith("t")) {
    float torque = cmd.substring(1).toFloat();
    setTorque(torque);
  } else {
    Serial.println("Unknown command");
  }
}
void processCanMessage(unsigned long rxId, byte len, byte* data) {
  // CyberGearからの応答を処理
  uint8_t motor_id = rxId & 0xFF;
  uint8_t comm_type = (rxId >> 24) & 0x1F;
  
  if(motor_id == MOTOR_ID) {
    switch(comm_type) {
      case COMM_MOTOR_FEEDBACK:
        if(len == 8) {
          // ステータスデータを解析
          parseStatusData(data);
        }
        break;
      default:
        Serial.print("Received response from motor ");
        Serial.print(motor_id);
        Serial.print(", comm_type: 0x");
        Serial.print(comm_type, HEX);
        Serial.print(", data: ");
        for(int i = 0; i < len; i++) {
          Serial.print(data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        break;
    }
  }
}
void parseStatusData(byte* data) {
  // CyberGearのステータスデータフォーマットに応じて解析
  // （実際のフォーマットはCyberGearの仕様書を参照）
  uint16_t pos_raw = (data[1] << 8) | data[0];
  uint16_t vel_raw = (data[3] << 8) | data[2];
  uint16_t torque_raw = (data[5] << 8) | data[4];
  
  // 生データから実際の値に変換（スケーリング係数は仕様書参照）
  current_position = float_from_uint(pos_raw, P_MIN, P_MAX, 16);
  current_velocity = float_from_uint(vel_raw, V_MIN, V_MAX, 16);
  current_torque = float_from_uint(torque_raw, T_MIN, T_MAX, 16);
  
  Serial.print("Status - Pos: ");
  Serial.print(current_position, 3);
  Serial.print(" rad, Vel: ");
  Serial.print(current_velocity, 3);
  Serial.print(" rad/s, Torque: ");
  Serial.print(current_torque, 3);
  Serial.println(" Nm");
}
void enableMotor() {
  byte data[8] = {0};
  uint32_t can_id = make_can_id(COMM_MOTOR_ENABLE, MASTER_ID, MOTOR_ID);
  
  if(CAN.sendMsgBuf(can_id, 1, 8, data) == CAN_OK) {
    Serial.println("Motor enabled");
  } else {
    Serial.println("Failed to enable motor");
  }
}
void disableMotor() {
  byte data[8] = {0};
  uint32_t can_id = make_can_id(COMM_MOTOR_DISABLE, MASTER_ID, MOTOR_ID);
  
  if(CAN.sendMsgBuf(can_id, 1, 8, data) == CAN_OK) {
    Serial.println("Motor disabled");
  } else {
    Serial.println("Failed to disable motor");
  }
}
void setMechanicalZero() {
  byte data[8] = {0};
  data[0] = 1; // マニュアル準拠：Byte[0]=1
  uint32_t can_id = make_can_id(COMM_SET_MECH_ZERO, MASTER_ID, MOTOR_ID);
  
  if(CAN.sendMsgBuf(can_id, 1, 8, data) == CAN_OK) {
    Serial.println("Mechanical zero set");
  } else {
    Serial.println("Failed to set mechanical zero");
  }
}
void setPosition(float pos, float vel, float kp, float kd) {
  byte data[8];
  
  // パラメータを制限
  pos = constrain(pos, P_MIN, P_MAX);
  vel = constrain(vel, V_MIN, V_MAX);
  kp = constrain(kp, KP_MIN, KP_MAX);
  kd = constrain(kd, KD_MIN, KD_MAX);
  
  // マニュアル準拠のデータパッキング（運動制御モード）
  uint16_t pos_raw = uint_from_float(pos, P_MIN, P_MAX, 16);
  uint16_t vel_raw = uint_from_float(vel, V_MIN, V_MAX, 16);
  uint16_t kp_raw = uint_from_float(kp, KP_MIN, KP_MAX, 16);
  uint16_t kd_raw = uint_from_float(kd, KD_MIN, KD_MAX, 16);
  
  data[0] = pos_raw & 0xFF;
  data[1] = (pos_raw >> 8) & 0xFF;
  data[2] = vel_raw & 0xFF;
  data[3] = (vel_raw >> 8) & 0xFF;
  data[4] = kp_raw & 0xFF;
  data[5] = (kp_raw >> 8) & 0xFF;
  data[6] = kd_raw & 0xFF;
  data[7] = (kd_raw >> 8) & 0xFF;
  
  // トルクは上位16ビットに配置（マニュアル準拠）
  uint16_t torque_raw = uint_from_float(0.0, T_MIN, T_MAX, 16);
  uint32_t can_id = make_can_id(COMM_MOTOR_CONTROL, torque_raw, MOTOR_ID);
  
  if(CAN.sendMsgBuf(can_id, 1, 8, data) == CAN_OK) {
    Serial.print("Position set to: ");
    Serial.print(pos, 3);
    Serial.println(" rad");
  } else {
    Serial.println("Failed to set position");
  }
}
// パラメータ書き込み関数
void writeParameter(uint16_t index, float value) {
  byte data[8] = {0};
  data[0] = index & 0xFF;
  data[1] = (index >> 8) & 0xFF;
  memcpy(&data[4], &value, 4);
  
  uint32_t can_id = make_can_id(COMM_PARAM_WRITE, MASTER_ID, MOTOR_ID);
  CAN.sendMsgBuf(can_id, 1, 8, data);
}

// モード設定関数
void setRunMode(uint8_t mode) {
  byte data[8] = {0};
  data[0] = PARAM_RUN_MODE & 0xFF;
  data[1] = (PARAM_RUN_MODE >> 8) & 0xFF;
  data[4] = mode;
  
  uint32_t can_id = make_can_id(COMM_PARAM_WRITE, MASTER_ID, MOTOR_ID);
  CAN.sendMsgBuf(can_id, 1, 8, data);
}

void setVelocity(float vel, float torque_limit) {
  vel = constrain(vel, V_MIN, V_MAX);
  torque_limit = constrain(torque_limit, T_MIN, T_MAX);
  
  // 速度モードに切り替え
  setRunMode(MODE_SPEED);
  delay(10);
  
  // 電流制限設定
  writeParameter(PARAM_LIMIT_CUR, 23.0); // 最大電流制限
  delay(10);
  
  // 速度コマンド設定
  writeParameter(PARAM_SPD_REF, vel);
  
  Serial.print("Velocity set to: ");
  Serial.print(vel, 3);
  Serial.println(" rad/s");
}
void setTorque(float torque) {
  torque = constrain(torque, T_MIN, T_MAX);
  
  // 電流モードに切り替え
  setRunMode(MODE_CURRENT);
  delay(10);
  
  // 電流コマンド設定（トルク制御）
  writeParameter(PARAM_IQ_REF, torque);
  
  Serial.print("Torque set to: ");
  Serial.print(torque, 3);
  Serial.println(" Nm");
}
void requestStatus() {
  // CyberGearでは定期的なフィードバックが自動で送られてくるため、
  // 特別なステータス要求は不要です
  // パラメータ読み出しの場合は COMM_PARAM_READ (17) を使用します
}
// ユーティリティ関数：floatを固定小数点に変換（マニュアル準拠）
uint16_t uint_from_float(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  if(x > x_max) x = x_max;
  else if(x < x_min) x = x_min;
  return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

// ユーティリティ関数：固定小数点をfloatに変換（マニュアル準拠）
float float_from_uint(uint16_t x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}
