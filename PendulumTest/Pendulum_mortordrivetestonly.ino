//モータードライバをArduinoに接続し、で正転逆転を繰り返すだけのコード

//モーター用変数
#define MOTOR1_FW 4   //モーターM1の前進用デジタルピン
#define MOTOR1_BW 5   //モーターM1の前進用デジタルピン
#define MOTOR1_PW 6   //モーターM1のスピード用アナログ(PWM)ピン
#define MOTOR2_FW 7   //モーターM2の前進用デジタルピン
#define MOTOR2_BW 8   //モーターM2の前進用デジタルピン
#define MOTOR2_PW 11  //モーターM2のスピード用アナログ(PWM)ピン
int power_abs = 100;  //電圧値の絶対値

void setup() {
  Serial.begin(115200);  //115200bps

  pinMode(MOTOR1_FW, OUTPUT);
  pinMode(MOTOR1_BW, OUTPUT);
  pinMode(MOTOR1_PW, OUTPUT);
  pinMode(MOTOR2_FW, OUTPUT);
  pinMode(MOTOR2_BW, OUTPUT);
  pinMode(MOTOR2_PW, OUTPUT);
}

void loop() {
  analogWrite(MOTOR1_PW, int(power_abs));
  digitalWrite(MOTOR1_FW, HIGH);
  digitalWrite(MOTOR1_BW, LOW);
  analogWrite(MOTOR2_PW, int(power_abs));
  digitalWrite(MOTOR2_FW, HIGH);
  digitalWrite(MOTOR2_BW, LOW);
  Serial.println("**forward**");
  delay(2000);

  analogWrite(MOTOR1_PW, int(power_abs));
  digitalWrite(MOTOR1_FW, LOW);
  digitalWrite(MOTOR1_BW, HIGH);
  analogWrite(MOTOR2_PW, int(power_abs));
  digitalWrite(MOTOR2_FW, LOW);
  digitalWrite(MOTOR2_BW, HIGH);
  Serial.println("**backward**");
  delay(2000);
}
